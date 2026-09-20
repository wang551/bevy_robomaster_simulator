use bevy::prelude::*;

use crate::components::{Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset};
use crate::config::{GimbalAxisPidConfig, SimulationConfig};

/// Absolute muzzle-frame aim target produced by an external auto-aim solver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GimbalAimTarget {
    pub yaw: f32,
    pub pitch: f32,
}

impl GimbalAimTarget {
    /// Invert [`Self::rotation`] on a live muzzle world rotation: recovers the
    /// `(yaw, pitch)` the PID is currently tracking. The gun barrel is the
    /// muzzle's local `+Y` (the `SHOT_DIRECTION` mount pitches it up 25° at
    /// rest, projectiles launch along `Vec3::Y`), so the probe is its antipode
    /// `-Y`: rolling the muzzle about the barrel leaves it untouched, and the
    /// extraction singularity needs the probe straight up/down — i.e. barrel
    /// elevation ±90°, which the ±45° gimbal pitch limit cannot reach. Probing
    /// `+Z` instead tracks an axis 90° below the barrel and flips the extracted
    /// yaw by 180° the moment the barrel dips under the horizon.
    pub fn from_muzzle_world(rotation: Quat) -> Self {
        let back = rotation * Vec3::NEG_Y;
        Self {
            yaw: back.x.atan2(back.z),
            pitch: -back.y.asin() - std::f32::consts::FRAC_PI_2,
        }
    }

    /// Solver-protocol angle deltas in degrees: positive yaw diff turns the same way
    /// positive solver yaw does (CCW seen from above), positive pitch diff raises the
    /// muzzle. Yaw wraps so the PID always takes the short way around.
    pub fn shifted_by_deg(self, yaw_diff_deg: f32, pitch_diff_deg: f32) -> Self {
        Self {
            yaw: wrap_angle(self.yaw + yaw_diff_deg.to_radians()),
            pitch: self.pitch + pitch_diff_deg.to_radians(),
        }
    }

    fn rotation(self) -> Quat {
        Quat::from_euler(EulerRot::YXZ, self.yaw, self.pitch, 0.0)
    }
}

/// One independent single-axis PID loop.
#[derive(Clone, Copy, Debug, Default)]
struct AxisPid {
    integral: f32,
    previous_error: f32,
}

impl AxisPid {
    /// Angular rate command (rad/s) for this axis given the current error.
    fn step(&mut self, error: f32, config: &GimbalAxisPidConfig, dt: f32) -> f32 {
        self.integral =
            (self.integral + error * dt).clamp(-config.integral_limit, config.integral_limit);
        let derivative = (error - self.previous_error) / dt;
        self.previous_error = error;

        let rate = error * config.kp + self.integral * config.ki + derivative * config.kd;
        rate.clamp(-config.max_rate, config.max_rate)
    }
}

/// Closed-loop state for tracking a solver target. Its presence on the gimbal *is*
/// the "auto-aim has a target" state: no tracker means no target and no PID output,
/// and a freshly inserted tracker starts from clean integrator/derivative state.
#[derive(Component, Clone, Copy, Debug)]
pub struct GimbalAimTracker {
    target: GimbalAimTarget,
    yaw: AxisPid,
    pitch: AxisPid,
}

impl GimbalAimTracker {
    pub fn new(target: GimbalAimTarget) -> Self {
        Self {
            target,
            yaw: AxisPid::default(),
            pitch: AxisPid::default(),
        }
    }

    pub fn target(&self) -> GimbalAimTarget {
        self.target
    }

    /// Update the setpoint while keeping the loop state, so a stream of commands
    /// drives one continuous controller instead of restarting every message.
    pub fn retarget(&mut self, target: GimbalAimTarget) {
        self.target = target;
    }
}

fn wrap_angle(angle: f32) -> f32 {
    let wrapped = (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU);
    wrapped - std::f32::consts::PI
}

/// Tracking error as `(yaw, pitch)` in the gimbal's own local frame, i.e. in the same
/// coordinates as `InfantryGimbal`. The solver target aims the *muzzle* in world space,
/// so the muzzle's fixed mount offset and the chassis rotation are divided out first —
/// measuring the error in world space instead flips the pitch sign for any mount whose
/// local axes disagree with the world axes.
fn tracking_error(
    target_rotation: Quat,
    gimbal_local: Quat,
    gimbal_world: Quat,
    muzzle_world: Quat,
) -> Vec2 {
    // World-space correction that would put the muzzle on target right now.
    let correction = target_rotation * muzzle_world.inverse();
    // Same correction expressed as the gimbal's desired local rotation.
    let desired_local = gimbal_local * gimbal_world.inverse() * correction * gimbal_world;

    let (desired_yaw, desired_pitch, _) = desired_local.to_euler(EulerRot::YXZ);
    let (current_yaw, current_pitch, _) = gimbal_local.to_euler(EulerRot::YXZ);

    Vec2::new(
        wrap_angle(desired_yaw - current_yaw),
        wrap_angle(desired_pitch - current_pitch),
    )
}

/// Replaces the direct pose snap that auto-aim used to apply: the solver target is
/// tracked by a rate-limited PID loop so the gimbal moves like an actuated axis.
pub fn gimbal_pid_controls(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    gimbal: Option<
        Single<
            (
                &mut Transform,
                &GlobalTransform,
                &mut InfantryGimbal,
                &mut GimbalAimTracker,
            ),
            (
                With<Controlled>,
                Without<InfantryChassis>,
                Without<InfantryLaunchOffset>,
            ),
        >,
    >,
    muzzle: Option<Single<&GlobalTransform, (With<InfantryLaunchOffset>, With<Controlled>)>>,
) {
    let (Some(gimbal), Some(muzzle)) = (gimbal, muzzle) else {
        return;
    };
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    let (mut gimbal_transform, gimbal_global, mut gimbal_data, mut tracker) = gimbal.into_inner();

    // Error is measured on the muzzle, so chassis motion shows up as tracking error
    // instead of being cancelled out by a one-shot correction.
    let error = tracking_error(
        tracker.target().rotation(),
        gimbal_transform.rotation,
        gimbal_global.rotation(),
        muzzle.rotation(),
    );
    let (yaw_error, pitch_error) = (error.x, error.y);
    let pid = &config.vehicle.gimbal_pid;
    let yaw_rate = tracker.yaw.step(yaw_error, &pid.yaw, dt);
    let pitch_rate = tracker.pitch.step(pitch_error, &pid.pitch, dt);

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);
    gimbal_data.local_yaw += yaw_rate * dt;
    gimbal_data.pitch = (gimbal_data.pitch + pitch_rate * dt).clamp(
        -config.vehicle.gimbal_pitch_limit,
        config.vehicle.gimbal_pitch_limit,
    );

    gimbal_transform.rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis(kp: f32, max_rate: f32) -> GimbalAxisPidConfig {
        GimbalAxisPidConfig {
            kp,
            ki: 0.0,
            kd: 0.0,
            integral_limit: 1.0,
            max_rate,
        }
    }

    #[test]
    fn proportional_rate_follows_error_sign_and_magnitude() {
        let mut pid = AxisPid::default();

        assert!((pid.step(0.2, &axis(10.0, 20.0), 0.01) - 2.0).abs() < 1e-5);
        assert!((pid.step(-0.1, &axis(10.0, 20.0), 0.01) + 1.0).abs() < 1e-5);
    }

    #[test]
    fn each_axis_uses_its_own_gains_and_rate_limit() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let yaw_config = axis(10.0, 20.0);
        let pitch_config = axis(4.0, 1.0);

        let yaw_rate = tracker.yaw.step(0.5, &yaw_config, 0.01);
        let pitch_rate = tracker.pitch.step(0.5, &pitch_config, 0.01);

        assert!((yaw_rate - 5.0).abs() < 1e-5);
        assert_eq!(pitch_rate, 1.0);
    }

    #[test]
    fn axis_loop_state_is_not_shared_between_yaw_and_pitch() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let config = axis(0.0, 20.0);

        tracker.yaw.step(1.0, &config, 0.5);

        assert_eq!(tracker.yaw.integral, 0.5);
        assert_eq!(tracker.pitch.integral, 0.0);
        assert_eq!(tracker.pitch.previous_error, 0.0);
    }

    #[test]
    fn integral_term_is_bounded_against_windup() {
        let mut config = axis(0.0, 20.0);
        config.ki = 1.0;
        let mut pid = AxisPid::default();

        for _ in 0..1000 {
            pid.step(1.0, &config, 0.01);
        }

        assert_eq!(pid.integral, config.integral_limit);
    }

    #[test]
    fn retarget_keeps_loop_state_so_command_stream_is_continuous() {
        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        tracker.yaw.step(0.1, &axis(10.0, 20.0), 0.01);
        let integral = tracker.yaw.integral;

        tracker.retarget(GimbalAimTarget {
            yaw: 0.5,
            pitch: 0.1,
        });

        assert_eq!(tracker.yaw.integral, integral);
        assert_eq!(tracker.target().yaw, 0.5);
    }

    #[test]
    fn error_points_the_same_way_as_the_local_angles_on_a_bare_gimbal() {
        let target = Quat::from_euler(EulerRot::YXZ, 0.3, -0.2, 0.0);
        let error = tracking_error(target, Quat::IDENTITY, Quat::IDENTITY, Quat::IDENTITY);

        assert!((error.x - 0.3).abs() < 1e-5);
        assert!((error.y + 0.2).abs() < 1e-5);
    }

    /// Guards the pitch sign. The gimbal's pitch axis is its *local* X, which points
    /// against world X once the gimbal has yawed past 90 degrees; an error read off the
    /// world-space correction is inverted there and drives pitch away from the target.
    #[test]
    fn pitch_error_keeps_its_sign_when_the_gimbal_faces_backwards() {
        let local = Quat::from_rotation_y(std::f32::consts::PI);
        let target = local * Quat::from_rotation_x(0.1);
        let error = tracking_error(target, local, local, local);

        assert!(error.x.abs() < 1e-5, "yaw leaked {}", error.x);
        assert!((error.y - 0.1).abs() < 1e-5, "pitch error was {}", error.y);
    }

    #[test]
    fn closed_loop_converges_through_a_mount_offset_and_chassis_yaw() {
        let chassis = Quat::from_rotation_y(2.8);
        let mount = Quat::from_euler(EulerRot::YXZ, 0.4, 0.25, 0.15);
        let goal = Quat::from_euler(EulerRot::YXZ, 0.3, -0.2, 0.0);
        let target = chassis * goal * mount;

        let mut tracker = GimbalAimTracker::new(GimbalAimTarget {
            yaw: 0.0,
            pitch: 0.0,
        });
        let yaw_config = axis(10.0, 20.0);
        let pitch_config = axis(6.0, 20.0);
        let mut local = Quat::IDENTITY;
        let dt = 1.0 / 120.0;

        for _ in 0..600 {
            let gimbal_world = chassis * local;
            let error = tracking_error(target, local, gimbal_world, gimbal_world * mount);
            let (mut yaw, mut pitch, _) = local.to_euler(EulerRot::YXZ);
            yaw += tracker.yaw.step(error.x, &yaw_config, dt) * dt;
            pitch += tracker.pitch.step(error.y, &pitch_config, dt) * dt;
            local = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
        }

        let (yaw, pitch, _) = local.to_euler(EulerRot::YXZ);
        assert!((yaw - 0.3).abs() < 1e-3, "yaw settled at {yaw}");
        assert!((pitch + 0.2).abs() < 1e-3, "pitch settled at {pitch}");
    }

    #[test]
    fn wrap_angle_takes_the_short_way_around() {
        assert!((wrap_angle(std::f32::consts::TAU + 0.2) - 0.2).abs() < 1e-5);
        assert!((wrap_angle(std::f32::consts::PI + 0.1) + std::f32::consts::PI - 0.1).abs() < 1e-5);
    }

    #[test]
    fn from_muzzle_world_roundtrips_the_target_parametrization() {
        // Pitch domain of the muzzle mount: barrel elevation − 90°, i.e. well
        // below zero everywhere the gimbal pitch limit can reach.
        for (yaw, pitch) in [
            (0.0f32, -0.349),
            (0.6, -1.134),
            (-1.2, -0.4),
            (2.8, -1.920),
            (-2.8, -0.349),
            (0.0, -2.8),
        ] {
            let target = GimbalAimTarget { yaw, pitch };
            let recovered = GimbalAimTarget::from_muzzle_world(target.rotation());

            let yaw_error = wrap_angle(recovered.yaw - yaw);
            assert!(
                yaw_error.abs() < 1e-5,
                "yaw {yaw} recovered as {}",
                recovered.yaw
            );
            assert!(
                (recovered.pitch - pitch).abs() < 1e-5,
                "pitch {pitch} recovered as {}",
                recovered.pitch
            );
        }
    }

    /// Regression test for the 180° yaw flip: the old `+Z` probe tracked an axis
    /// 90° below the barrel, whose `atan2` yaw jumped by 180° exactly when the
    /// barrel crossed the horizon (muzzle-frame pitch −90°, gimbal pitch ≈ −25°
    /// with the 25° mount). The barrel-antipode probe must stay continuous
    /// through that crossing.
    #[test]
    fn from_muzzle_world_stays_continuous_across_the_horizon() {
        for pitch_deg in [-100.0f32, -95.0, -90.0, -85.0, -80.0] {
            let pitch = pitch_deg.to_radians();
            let muzzle = Quat::from_euler(EulerRot::YXZ, 0.8, pitch, 0.0);
            let extracted = GimbalAimTarget::from_muzzle_world(muzzle);

            assert!(
                (wrap_angle(extracted.yaw - 0.8)).abs() < 1e-5,
                "yaw flipped to {} at pitch {pitch_deg}°",
                extracted.yaw
            );
            assert!(
                (extracted.pitch - pitch).abs() < 1e-5,
                "pitch {pitch} recovered as {}",
                extracted.pitch
            );
        }
    }

    /// A mount that rolls the muzzle about the barrel axis must not shift the extracted
    /// angles: the diff base has to reflect where the barrel actually points, not how
    /// it is rolled, so sender-side diffs reconstruct their intended absolute aim.
    #[test]
    fn from_muzzle_world_ignores_roll_about_the_aim_axis() {
        let target = GimbalAimTarget {
            yaw: 0.9,
            pitch: -0.25,
        };
        let rolled = target.rotation() * Quat::from_rotation_y(0.6);

        let recovered = GimbalAimTarget::from_muzzle_world(rolled);

        assert!((wrap_angle(recovered.yaw - 0.9)).abs() < 1e-5);
        assert!((recovered.pitch + 0.25).abs() < 1e-5);
    }

    #[test]
    fn shifted_by_deg_accumulates_and_wraps_yaw() {
        let base = GimbalAimTarget {
            yaw: 3.0,
            pitch: 0.1,
        };

        let up = base.shifted_by_deg(30.0, 10.0);
        assert!((wrap_angle(up.yaw - (3.0 + 30.0f32.to_radians()))).abs() < 1e-5);
        assert!((up.pitch - (0.1 + 10.0f32.to_radians())).abs() < 1e-5);

        let down = base.shifted_by_deg(-30.0, -10.0);
        assert!(down.yaw.abs() < std::f32::consts::PI);
        assert!((down.pitch - (0.1 - 10.0f32.to_radians())).abs() < 1e-5);
    }
}
