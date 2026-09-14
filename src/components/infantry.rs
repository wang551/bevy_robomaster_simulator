use bevy::prelude::*;

use crate::robomaster::prelude::{RobotConfig, Team};

#[derive(Component)]
pub struct Controlled;

#[derive(Component)]
pub struct Infantry {
    pub team: Team,
    pub config: RobotConfig,
}

impl Infantry {
    pub const fn new(team: Team, config: RobotConfig) -> Self {
        Self { team, config }
    }
}

#[derive(Component, Default)]
pub struct InfantryChassis {
    pub yaw: f32,
    pub yaw_velocity: f32,
    pub roll: f32,
    pub pitch: f32,
}

#[derive(Component, Default)]
pub struct InfantryGimbal {
    pub local_yaw: f32,
    pub pitch: f32,
}

#[derive(Component)]
pub struct InfantryViewOffset;

#[derive(Component)]
pub struct InfantryLaunchOffset;

#[derive(Component)]
pub struct SlapperInfantry;

/// Marker for the currently active (controlled) SlapperInfantry
#[derive(Component)]
pub struct ActiveSlapper;

/// Latest `/cmd_vel` (geometry_msgs/Twist) command for the controlled chassis.
///
/// Semantics follow REP-103 body-frame convention: `linear.x` forward,
/// `linear.y` left [m/s], `angular.z` yaw rate [rad/s, CCW positive], all
/// relative to the chassis (base_link) frame. Filled by the ROS2 subscriber;
/// consumed by `vehicle_controls` while fresh (see `active_target`).
#[derive(Resource, Debug, Default, Clone, Copy)]
pub struct NavCmdVel {
    /// ROS body-frame linear velocity: x forward, y left [m/s].
    pub linear: Vec2,
    /// Yaw rate around the up axis [rad/s], CCW positive.
    pub angular_z: f32,
    last_cmd: Option<std::time::Instant>,
}

impl NavCmdVel {
    /// Duration after the last received message before the command is
    /// considered lost; matches Nav2's default `cmd_vel_timeout`.
    pub const TIMEOUT: std::time::Duration = std::time::Duration::from_millis(500);

    pub fn update(&mut self, linear: Vec2, angular_z: f32) {
        self.linear = linear;
        self.angular_z = angular_z;
        self.last_cmd = Some(std::time::Instant::now());
    }

    #[cfg(test)]
    pub(crate) fn age_last_command(&mut self, duration: std::time::Duration) {
        if let Some(last) = self.last_cmd.as_mut() {
            *last = *last - duration;
        }
    }

    /// Chassis velocity target while navigation owns the chassis:
    /// - `Some(target)` while a command is fresh;
    /// - `Some(zero)` after the command is lost but the chassis is still
    ///   moving, so it actively brakes instead of coasting on friction;
    /// - `None` once stopped, handing control back to keyboard input.
    pub fn active_target(&self, body_speed: f32, yaw_speed: f32) -> Option<(Vec2, f32)> {
        match self.last_cmd {
            Some(last) if last.elapsed() < Self::TIMEOUT => Some((self.linear, self.angular_z)),
            _ => {
                const STOP_EPSILON: f32 = 0.05;
                if self.last_cmd.is_some()
                    && (body_speed > STOP_EPSILON || yaw_speed.abs() > STOP_EPSILON)
                {
                    Some((Vec2::ZERO, 0.0))
                } else {
                    None
                }
            }
        }
    }
}
