use crate::arc_mutex;
use crate::capture::CaptureSource;
use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::components::{
    Controlled, Infantry, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, NavCmdVel,
    SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{
    ArmorParts, ArmorRoot, PowerRune, RuneIndex, TechCore, tech_core_state_json,
};
use crate::ros2::capture::{RosCaptureContext, RosCapturePlugin};
use crate::ros2::livox::{RosLivoxContext, RosLivoxPlugin};
use crate::ros2::prelude::AverageRateLimiter;
use crate::ros2::prelude::transform;
use crate::ros2::topic::*;
use crate::systems::{
    ChassisObservationFrame, GimbalAimTarget, GimbalAimTracker, bevy_local_to_body,
    chassis_world_angular_velocity, projectile_launch,
};
use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::ecs::system::RunSystemOnce;
use bevy::image::BevyDefault;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use r2r::ClockType::SystemTime;
use r2r::geometry_msgs::msg::{
    Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
};
use r2r::nav_msgs::msg::Odometry;
use r2r::sensor_msgs::msg::Imu;
use r2r::std_msgs::msg::{ColorRGBA, String as RosString};
use r2r::visualization_msgs::msg::Marker;
use r2r::{Clock, Context, Node, std_msgs::msg::Header, tf2_msgs::msg::TFMessage};
use std::collections::HashMap;
use std::f32::consts::PI;
use std::time::Duration;
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    thread::{self, JoinHandle},
};

macro_rules! res_unwrap {
    ($res:tt) => {
        $res.0.lock().unwrap()
    };
}

#[derive(Resource, Deref, DerefMut)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource, Deref, DerefMut)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Resource, Deref, DerefMut)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

#[derive(Resource, Deref, DerefMut)]
struct FireRateLimiter(AverageRateLimiter);

#[derive(Resource, Deref, DerefMut)]
struct TechCoreStateRateLimiter(AverageRateLimiter);

#[derive(Resource, Deref, DerefMut)]
struct CmdLogRateLimiter(AverageRateLimiter);

#[derive(Resource, Deref, DerefMut)]
struct CmdVelLogRateLimiter(AverageRateLimiter);

#[derive(Resource, Deref, DerefMut)]
struct OdomRateLimiter(AverageRateLimiter);

#[derive(Resource, Deref, DerefMut)]
struct ImuRateLimiter(AverageRateLimiter);

/// Accelerometers report specific force: at rest the sensor reads +g along
/// body-up. FAST-LIO-style LIO relies on this convention for gravity alignment,
/// so the kinematic acceleration gets the offset added before publishing.
const IMU_GRAVITY_MPS2: f32 = 9.81;

fn imu_accel_with_gravity(accel: Vec3) -> Vec3 {
    accel + Vec3::new(0.0, 0.0, IMU_GRAVITY_MPS2)
}

/// Body-frame z yaw rate of the chassis: the physics root's spin plus the
/// kinematic yaw rate about the root's local up axis (the axis the BASE yaw
/// rotates around in the YXZ euler chain), expressed in base_link.
fn body_yaw_rate(
    base_rotation: Quat,
    root_rotation: Quat,
    root_angular: Vec3,
    yaw_velocity: f32,
) -> f32 {
    let world = chassis_world_angular_velocity(root_rotation, root_angular, yaw_velocity);
    // `base_rotation.inverse() * world` is expressed in base_link's Bevy local
    // axes (x right, y up, z back); remap to REP-103 body axes and take the up
    // component, so the twist's angular.z is the yaw rate about body up.
    bevy_local_to_body(base_rotation.inverse() * world).z
}

/// 36-element covariance with x/y/z (index 0/7/14) and yaw (index 35) on the
/// diagonal; the roll/pitch slots a planar robot never observes get `unused`
/// so consumers treat them as uninformative. Off-diagonal terms stay zero:
/// a full matrix with large constant fill-in is singular, and consumers
/// (RTAB-Map link information) invert this matrix.
fn planar_covariance(xyz: f64, yaw: f64, unused: f64) -> Vec<f64> {
    let mut covariance = vec![0.0; 36];
    covariance[0] = xyz;
    covariance[7] = xyz;
    covariance[14] = xyz;
    covariance[21] = unused;
    covariance[28] = unused;
    covariance[35] = yaw;
    covariance
}

/// `sensor_msgs/Imu` covariances are fixed `double[9]` 3x3 matrices (r2r
/// asserts the length at publish time), so the diagonal fills indices 0/4/8.
fn diagonal_covariance_3x3(value: f64) -> Vec<f64> {
    let mut covariance = vec![0.0; 9];
    covariance[0] = value;
    covariance[4] = value;
    covariance[8] = value;
    covariance
}

macro_rules! tf_tree {
    (stamp: $stamp:expr;$root:literal { $($content:tt)* }) => {{
        let stamp = $stamp;
        let mut transform_stamped = vec![];
        let _parent = $root;
        let _current = $root;
        tf_tree!(@frame transform_stamped, stamp, _parent, _current, $($content)*);

        transform_stamped
    }};

    (@header $stamp:ident, $current:ident) => {
        Header {
            stamp: $stamp.clone(),
            frame_id: $current.to_string(),
        }
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident,
        $curr_name:literal as ($translation:expr, $rotation:expr) $(for $pub_:ident)?
        {$($children:tt)*}
        $($remaining:tt)*
    ) => {
        {
            let $parent = &$current;
            let $current = $curr_name;
            let frame_translation = $translation;
            let frame_rotation = $rotation;
            $crate::add_tf_frame!($tf_vec, tf_tree!(@header $stamp, $parent), $current, frame_translation, frame_rotation);
            $(
                $pub_.publish($crate::pose!(tf_tree!(@header $stamp, $parent), frame_translation, frame_rotation));
            )*
            tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($children)*);
        }
        tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($remaining)*);
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident,
    $(let $p_name:ident = $p_expr:expr;)*
        for ($($elem:tt),+$(,)?) in $iter:ident {
            $(let $name:ident = $expr:expr;)*
            pub $curr_name:ident as ($translation:expr, $rotation:expr) $(for $pub_:ident)?;
            $($children:tt)*
        }
        $($remaining:tt)*
    ) => {
        $(let $p_name = $p_expr;)*
        for ($($elem),+) in $iter {
            $(let $name = $expr;)*
            let $parent = &$current;
            let $current = $curr_name;
            let frame_translation = $translation;
            let frame_rotation = $rotation;
            $crate::add_tf_frame!($tf_vec, tf_tree!(@header $stamp, $parent), $current, frame_translation, frame_rotation);
            $(
                $pub_.publish($crate::pose!(tf_tree!(@header $stamp, $parent), frame_translation, frame_rotation));
            )*
            tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($children)*);
        }
        tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($remaining)*);
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident, $(;)? $(,)? $({})?) => { };
}

fn capture_rune(
    camera: Single<&GlobalTransform, With<CaptureSource>>,
    gimbal: Single<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>,
    muzzle_offset: Single<
        (&GlobalTransform, &Transform),
        (With<InfantryLaunchOffset>, With<Controlled>),
    >,
    chassis: Single<&GlobalTransform, (With<Controlled>, With<InfantryChassis>)>,

    runes: Query<(Entity, &GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    clock: ResMut<RoboMasterClock>,
    tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    muzzle_pose_pub: ResMut<TopicPublisher<MuzzlePoseTopic>>,
    camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
    center: Query<(Entity, &GlobalTransform)>,
    armor_parts: Query<&ArmorParts>,
    armor: Query<(Entity, &GlobalTransform, &ArmorRoot)>,
    marker_pub: ResMut<TopicPublisher<OutpostMarkerTopic>>,
) {
    let cam_transform = camera.into_inner();
    let gimbal = gimbal.into_inner();
    let cam_rel = cam_transform.reparented_to(gimbal);
    let muzzle_rel = muzzle_offset.0.reparented_to(gimbal);
    let chassis_global = chassis.into_inner();
    let gimbal_rel = gimbal.reparented_to(chassis_global);

    // Avoid per-frame `info!` logging (it will cap FPS under ROS2 builds).
    debug!(
        "[ROS2] ODOM pos: [{:.4}, {:.4}, {:.4}]",
        gimbal.translation().x,
        gimbal.translation().y,
        gimbal.translation().z
    );
    debug!(
        "[ROS2] CAMERA_REL pos: [{:.4}, {:.4}, {:.4}]",
        cam_rel.translation.x, cam_rel.translation.y, cam_rel.translation.z
    );
    let mut targets = targets.into_iter().fold(
        HashMap::<Entity, Vec<(String, Transform)>>::new(),
        |mut map, (tf, target, name)| {
            // only use one target
            if !name.contains("_ACTIVATED") {
                return map;
            }
            let Ok((rune_entity, rune_tf, rune)) = runes.get(target.rune) else {
                return map;
            };
            map.entry(rune_entity).or_default().push((
                format!("power_rune_{:?}_{:?}", rune.mode(), target.target)
                    .to_string()
                    .to_lowercase(),
                tf.reparented_to(rune_tf),
            ));
            map
        },
    );

    debug!(
        "[ROS2] MUZZLE pos: [{:.4}, {:.4}, {:.4}]",
        muzzle_rel.translation.x, muzzle_rel.translation.y, muzzle_rel.translation.z
    );
    let rot = (gimbal.rotation()
        * muzzle_offset.1.rotation
        * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0))
    .to_euler(EulerRot::ZXY);
    debug!(
        "[ROS2] GIMBAL rpy: [{:.4}, {:.4}, {:.4}]",
        rot.0.to_degrees(),
        rot.1.to_degrees(),
        rot.2.to_degrees()
    );

    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    // Not part of the TF tree on purpose: gimbal_link now hangs under
    // base_link (which rotates with chassis yaw), while this topic keeps
    // exposing the world-frame gimbal orientation the auto-aim stack anchors
    // on. The rotation construction is identical to the pre-restructure value.
    gimbal_pose_pub.publish(crate::pose!(
        Header {
            stamp: stamp.clone(),
            frame_id: "odom".to_string()
        },
        gimbal.translation(),
        gimbal.rotation()
            * muzzle_offset.1.rotation
            * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0)
    ));

    // odom is a root: map->odom belongs to the SLAM stack (REP-105), so the
    // simulator no longer publishes that edge. The map debug frames have no
    // common ancestor with the odom tree and stay in a second root.
    //
    // base_link carries the chassis' full world rotation — the physics root's
    // spin composed with the kinematic BASE yaw — not just the kinematic yaw.
    // The camera that renders the depth cloud rotates with the physics root
    // (`update_camera_follow`), and every child edge below is taken relative
    // to this same transform, so only the full rotation keeps /tf identical
    // to what the sensors actually observe. The collision-driven root spin
    // used to leak into the point cloud while staying invisible in /odom.
    let mut transform_stamped = tf_tree! {
        stamp: stamp.clone();

        "odom" {
            "base_link" as (chassis_global.translation(), chassis_global.rotation()) for odom_pose_pub {
                "gimbal_link" as (gimbal_rel.translation, gimbal_rel.rotation * muzzle_offset.1.rotation * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0)) {
                    "muzzle" as (muzzle_rel.translation, Quat::IDENTITY) {
                        "muzzle_link" as (Vec3::ZERO, Quat::IDENTITY) for muzzle_pose_pub{}
                    }
                    "camera_link" as (cam_rel.translation, Quat::IDENTITY) for camera_pose_pub {
                        "camera_optical_frame" as (Vec3::ZERO, Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)) {}
                    }
                }
            }
        }
    };
    transform_stamped.extend(tf_tree! {
        stamp: stamp.clone();

        "map" {
            for (rune_entity, transform, rune) in runes {
                let name = format!("power_rune_{:?}", rune.mode()).to_string().to_lowercase();
                let tf = transform.compute_transform();
                pub name as (tf.translation, tf.rotation);
                let targets = targets.remove(&rune_entity).unwrap_or_default();
                for (name, tf) in targets {
                    pub name as (tf.translation, tf.rotation);
                }
            }
            for (entity, _transform, armor) in armor {
                let name = format!("armor_{:?}", armor.id.as_usize())
                    .to_string()
                    .to_lowercase();
                let tf = center
                    .get(armor_parts.get(entity).unwrap().marker())
                    .unwrap()
                    .1
                    .compute_transform();
                pub name as (tf.translation, tf.rotation);
            }
        }
    });
    for (entity, tf, armor) in armor {
        let mut tff = center
            .get(armor_parts.get(entity).unwrap().marker())
            .unwrap()
            .1
            .compute_transform();
        tff.rotation = tf.rotation() * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, -PI / 2.0);
        let tf = transform(tff);
        marker_pub.publish(Marker {
            header: Header {
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            }
            .clone(),
            ns: "armors".to_string(),
            id: armor.id.as_usize() as i32,
            type_: Marker::CUBE as i32,
            action: Marker::ADD as i32,
            pose: Pose {
                position: Point {
                    x: tf.translation.x,
                    y: tf.translation.y,
                    z: tf.translation.z,
                },
                orientation: tf.rotation,
            },
            scale: Vector3 {
                x: 0.03,
                y: 0.15,
                z: 0.125,
            },
            color: ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 0.0,
            },
            lifetime: r2r::builtin_interfaces::msg::Duration {
                sec: 0,
                nanosec: 300000000,
            },
            frame_locked: false,
            points: vec![],
            colors: vec![],
            texture_resource: "".to_string(),
            texture: Default::default(),
            uv_coordinates: vec![],
            text: "".to_string(),
            mesh_resource: "".to_string(),
            mesh_file: Default::default(),
            mesh_use_embedded_materials: false,
        });
    }

    tf_publisher.publish(TFMessage {
        transforms: transform_stamped,
    });
}

fn process_subscription(
    time: Res<Time>,
    mut commands: Commands,
    gimbal_cmd: ResMut<TopicSubscriber<GimbalCmdTopic>>,
    mut fire_rate_limiter: ResMut<FireRateLimiter>,
    mut cmd_log_limiter: ResMut<CmdLogRateLimiter>,
    gimbal: Single<
        (Entity, Option<&mut GimbalAimTracker>),
        (
            With<Controlled>,
            With<InfantryGimbal>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
    muzzle: Single<&GlobalTransform, (With<InfantryLaunchOffset>, With<Controlled>)>,
) {
    let (gimbal_entity, mut tracker) = gimbal.into_inner();
    fire_rate_limiter.tick(time.delta());
    cmd_log_limiter.tick(time.delta());
    // Commands are angle deltas re-based on the muzzle's current pointing, so the
    // messages drained in one frame must sum into a single step; applying them one by
    // one against the same base would keep only the last delta.
    let (mut yaw_diff_sum, mut pitch_diff_sum) = (0.0_f32, 0.0_f32);
    let mut has_command = false;
    loop {
        let Ok(Some(cmd)) = gimbal_cmd.try_recv() else {
            break;
        };
        has_command = true;
        // Diagnostic for the rm_sim_bridge round-trip: echo exactly what the
        // message carried so sent-vs-received field values can be diffed live.
        if cmd_log_limiter.allow() {
            info!(
                "[ROS2] GimbalCmd yaw={:.3} pitch={:.3} yaw_diff={:.3} pitch_diff={:.3} distance={:.3} fire={}",
                cmd.yaw, cmd.pitch, cmd.yaw_diff, cmd.pitch_diff, cmd.distance, cmd.fire_advice
            );
        }
        // No solution from the solver: drop the target so the PID loop stops driving.
        if cmd.distance == -1.0 {
            commands.entity(gimbal_entity).remove::<GimbalAimTracker>();
            return;
        }
        if cmd.fire_advice {
            if fire_rate_limiter.allow() {
                commands.queue(|w: &mut World| {
                    w.run_system_once(projectile_launch).unwrap();
                });
            }
        }

        yaw_diff_sum += cmd.yaw_diff as f32;
        pitch_diff_sum += cmd.pitch_diff as f32;
    }
    if !has_command {
        return;
    }

    let target = GimbalAimTarget::from_muzzle_world(muzzle.rotation())
        .shifted_by_deg(yaw_diff_sum, pitch_diff_sum);
    match tracker.as_mut() {
        Some(tracker) => tracker.retarget(target),
        None => {
            commands
                .entity(gimbal_entity)
                .insert(GimbalAimTracker::new(target));
        }
    }
}

fn process_cmd_vel(
    time: Res<Time>,
    cmd_vel: ResMut<TopicSubscriber<CmdVelTopic>>,
    mut nav: ResMut<NavCmdVel>,
    mut log_limiter: ResMut<CmdVelLogRateLimiter>,
) {
    log_limiter.tick(time.delta());
    loop {
        let Ok(Some(cmd)) = cmd_vel.try_recv() else {
            return;
        };
        // Same 2Hz echo pattern as the GimbalCmd diagnostic so commanded vs
        // observed chassis motion can be diffed live without capping FPS.
        if log_limiter.allow() {
            info!(
                "[ROS2] cmd_vel linear=({:.3}, {:.3}) angular.z={:.3}",
                cmd.linear.x, cmd.linear.y, cmd.angular.z
            );
        }
        nav.update(
            Vec2::new(cmd.linear.x as f32, cmd.linear.y as f32),
            cmd.angular.z as f32,
        );
    }
}

fn publish_odometry(
    time: Res<Time>,
    clock: ResMut<RoboMasterClock>,
    mut limiter: ResMut<OdomRateLimiter>,
    odom_pub: ResMut<TopicPublisher<OdomTopic>>,
    chassis: Single<
        (&GlobalTransform, &InfantryChassis),
        (With<Controlled>, With<InfantryChassis>),
    >,
    root: Single<
        (&GlobalTransform, &LinearVelocity, &AngularVelocity),
        (With<Infantry>, With<Controlled>),
    >,
) {
    limiter.tick(time.delta());
    if !limiter.allow() {
        return;
    }
    let (chassis_global, chassis_data) = chassis.into_inner();
    let (root_global, linear, angular) = root.into_inner();
    // Pose/twist are expressed against the chassis' full world rotation — the
    // physics root's spin composed with the kinematic BASE yaw. The yaw rate
    // combines the root's physical spin with the kinematic yaw rate about the
    // root's local up axis (the axis the BASE yaw rotates around), so the
    // twist integrates to the published pose and the cmd_vel executor (which
    // drives on this same world rotation) sees a consistent feedback loop.
    let rotation = chassis_global.rotation();
    let linear_body = bevy_local_to_body(rotation.inverse() * linear.0);
    let yaw_rate_body = body_yaw_rate(
        rotation,
        root_global.rotation(),
        angular.0,
        chassis_data.yaw_velocity,
    );

    // Pose must match the odom->base_link /tf edge bit-for-bit: full chassis
    // world rotation through the same Bevy->ROS alignment helper.
    let aligned = transform(
        Transform::IDENTITY
            .with_translation(chassis_global.translation())
            .with_rotation(rotation),
    );

    odom_pub.publish(Odometry {
        header: Header {
            stamp: Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap()),
            frame_id: "odom".to_string(),
        },
        child_frame_id: "base_link".to_string(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: aligned.translation.x,
                    y: aligned.translation.y,
                    z: aligned.translation.z,
                },
                orientation: aligned.rotation,
            },
            covariance: planar_covariance(0.01, 0.01, 1e6),
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: linear_body.x as f64,
                    y: linear_body.y as f64,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: yaw_rate_body as f64,
                },
            },
            covariance: planar_covariance(0.01, 0.01, 1e6),
        },
    });
}

fn publish_imu(
    time: Res<Time>,
    clock: ResMut<RoboMasterClock>,
    mut limiter: ResMut<ImuRateLimiter>,
    imu_pub: ResMut<TopicPublisher<ImuTopic>>,
    observation: Res<ChassisObservationFrame>,
) {
    limiter.tick(time.delta());
    if !limiter.allow() {
        return;
    }

    let [orientation_x, orientation_y, orientation_z, orientation_w] = Quat::from_euler(
        EulerRot::XYZ,
        observation.rpy_rad.x,
        observation.rpy_rad.y,
        observation.rpy_rad.z,
    )
    .to_array();
    let accel = imu_accel_with_gravity(observation.accel_xyz_mps2);
    imu_pub.publish(Imu {
        header: Header {
            stamp: Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap()),
            frame_id: "base_link".to_string(),
        },
        orientation: Quaternion {
            x: orientation_x as f64,
            y: orientation_y as f64,
            z: orientation_z as f64,
            w: orientation_w as f64,
        },
        orientation_covariance: diagonal_covariance_3x3(0.01),
        angular_velocity: Vector3 {
            x: observation.gyro_xyz_radps.x as f64,
            y: observation.gyro_xyz_radps.y as f64,
            z: observation.gyro_xyz_radps.z as f64,
        },
        angular_velocity_covariance: diagonal_covariance_3x3(0.01),
        linear_acceleration: Vector3 {
            x: accel.x as f64,
            y: accel.y as f64,
            z: accel.z as f64,
        },
        linear_acceleration_covariance: diagonal_covariance_3x3(0.02),
    });
}

fn publish_tech_core_state(
    time: Res<Time>,
    clock: Res<RoboMasterClock>,
    mut limiter: ResMut<TechCoreStateRateLimiter>,
    cores: Query<&TechCore>,
    state_pub: Res<TopicPublisher<TechCoreStateTopic>>,
) {
    limiter.tick(time.delta());
    if !limiter.allow() {
        return;
    }

    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    state_pub.publish(RosString {
        data: tech_core_state_json(
            stamp.sec,
            stamp.nanosec,
            time.elapsed_secs_f64(),
            cores.iter(),
        ),
    });
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.store(true, Ordering::Release);
        if let Some(handle) = handle_res.take() {
            info!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => info!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => error!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let sim_config = app
            .world()
            .get_resource::<SimulationConfig>()
            .cloned()
            .unwrap_or_default();
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));

        register_pub(signal_arc.clone(), app, &mut node);
        register_sub(signal_arc.clone(), app, &mut node);

        let camera_info = app
            .world_mut()
            .remove_resource::<TopicPublisher<CameraInfoTopic>>()
            .unwrap();
        let image_raw = app
            .world_mut()
            .remove_resource::<TopicPublisher<ImageRawTopic>>()
            .unwrap();
        let image_compressed = app
            .world_mut()
            .remove_resource::<TopicPublisher<ImageCompressedTopic>>()
            .unwrap();
        let livox_pointcloud = app
            .world_mut()
            .remove_resource::<TopicPublisher<LivoxPointCloudTopic>>()
            .unwrap();

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());
        let fov_y = sim_config.camera.fov.to_radians();
        let color_capture_config = CaptureConfig {
            width: sim_config.capture.color.width,
            height: sim_config.capture.color.height,
            texture_format: TextureFormat::bevy_default(),
            frame_kind: CapturedFrameKind::Rgb8,
        };
        let depth_capture_config = CaptureConfig {
            width: sim_config.capture.depth.width,
            height: sim_config.capture.depth.height,
            texture_format: TextureFormat::Depth32Float,
            frame_kind: CapturedFrameKind::Depth32F,
        };
        let publish_freq = sim_config.livox_ros.publish_freq.max(0.1);
        let points_per_publish =
            ((sim_config.livox_ros.points_per_second as f32) / publish_freq).max(1.0) as usize;

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(StopSignal(signal_arc.clone()))
            .insert_resource(FireRateLimiter(AverageRateLimiter::from_hz(10.0)))
            .insert_resource(TechCoreStateRateLimiter(AverageRateLimiter::from_hz(20.0)))
            .insert_resource(CmdLogRateLimiter(AverageRateLimiter::from_hz(2.0)))
            .insert_resource(CmdVelLogRateLimiter(AverageRateLimiter::from_hz(2.0)))
            // `from_hz` asserts hz > 0; clamp so a config value of 0 (documented
            // elsewhere as "no limit") degrades to 0.1 Hz instead of a startup
            // panic that kills the whole app before any publisher is created.
            .insert_resource(OdomRateLimiter(AverageRateLimiter::from_hz(
                sim_config.ros2.odom_hz.max(0.1),
            )))
            .insert_resource(ImuRateLimiter(AverageRateLimiter::from_hz(
                sim_config.ros2.imu_hz.max(0.1),
            )))
            .add_plugins(RosCapturePlugin {
                config: color_capture_config,
                context: RosCaptureContext {
                    clock: clock.clone(),
                    fov_y,
                    publish_compressed: sim_config.ros2.publish_compressed,
                    publish_period_ns: if sim_config.ros2.publish_hz > 0.0 {
                        (1_000_000_000.0 / sim_config.ros2.publish_hz) as u64
                    } else {
                        0
                    },
                    last_publish_ns: Arc::new(AtomicU64::new(0)),
                    camera_info,
                    image_raw,
                    image_compressed,
                },
            })
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(
                Update,
                process_subscription
                    .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
            )
            .add_systems(Update, process_cmd_vel)
            .add_systems(Update, capture_rune.after(TransformSystems::Propagate))
            .add_systems(Update, publish_odometry.after(TransformSystems::Propagate))
            .add_systems(Update, publish_imu)
            .add_systems(Update, publish_tech_core_state)
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(1));
                }
            }))));

        if sim_config.livox_ros.enabled {
            app.add_plugins(RosLivoxPlugin {
                config: depth_capture_config,
                context: RosLivoxContext {
                    clock,
                    frame_id: sim_config.livox_ros.frame_id.clone(),
                    fov_y,
                    near: sim_config.capture.depth.near,
                    far: sim_config.capture.depth.far,
                    publish_period_ns: (1_000_000_000.0 / publish_freq) as u64,
                    points_per_publish,
                    line_num: sim_config.livox_ros.line_num.max(1),
                    tag_default: sim_config.livox_ros.tag_default,
                    intensity_default: sim_config.livox_ros.intensity_default,
                    pointcloud: livox_pointcloud,
                    last_publish_ns: Arc::new(AtomicU64::new(0)),
                },
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planar_covariance_sets_planar_slots_and_marks_unused() {
        let covariance = planar_covariance(0.01, 0.02, 1e6);
        assert_eq!(covariance[0], 0.01);
        assert_eq!(covariance[7], 0.01);
        assert_eq!(covariance[14], 0.01);
        assert_eq!(covariance[35], 0.02);
        assert_eq!(covariance[21], 1e6);
        assert_eq!(covariance[28], 1e6);
        assert_eq!(covariance.len(), 36);
        // Only the unobserved roll/pitch slots carry the unused variance.
        // Everything else is zero: off-diagonal fill-in made the 6x6 matrix
        // singular, and consumers that invert it into an information matrix
        // (RTAB-Map) died on "Linear information X should not be null".
        assert_eq!(covariance.iter().filter(|v| **v == 1e6).count(), 2);
        let off_diagonal_nonzero = covariance
            .iter()
            .enumerate()
            .any(|(i, v)| i % 6 != i / 6 && *v != 0.0);
        assert!(!off_diagonal_nonzero);
        // Diagonal, so invertible iff every diagonal entry is positive.
        assert!(covariance.iter().step_by(7).all(|v| *v > 0.0));
    }

    #[test]
    fn yaw_only_quaternion_keeps_angle_and_sign_through_ros_alignment() {
        // The Bevy->ROS alignment must map a yaw-only Bevy rotation to the
        // same yaw angle in the ROS frame (both conventions are CCW-positive
        // about the vertical axis).
        for yaw_deg in [0.0f32, 30.0, -120.0, 179.0] {
            let aligned = transform(
                Transform::IDENTITY.with_rotation(Quat::from_rotation_y(yaw_deg.to_radians())),
            );
            let rotation = Quat::from_xyzw(
                aligned.rotation.x as f32,
                aligned.rotation.y as f32,
                aligned.rotation.z as f32,
                aligned.rotation.w as f32,
            );
            let (yaw_ros, _, _) = rotation.to_euler(EulerRot::ZYX);
            assert!(
                (yaw_ros - yaw_deg.to_radians()).abs() < 1e-4,
                "yaw {yaw_deg} deg mapped to {} rad",
                yaw_ros.to_degrees()
            );
        }
    }

    #[test]
    fn composed_chassis_rotation_keeps_yaw_through_ros_alignment() {
        // base_link publishes the composed physics-root x kinematic-yaw
        // rotation. The alignment must keep its yaw intact so SLAM/Nav2 see
        // the heading the render — and the cmd_vel executor — actually use,
        // independent of any kinematic tilt carried in the same quaternion.
        for yaw_deg in [0.0f32, 30.0, -120.0] {
            for tilt_deg in [0.0f32, 5.0, -17.0] {
                let base = Quat::from_euler(
                    EulerRot::YXZ,
                    yaw_deg.to_radians(),
                    tilt_deg.to_radians(),
                    0.0,
                );
                let aligned = transform(Transform::IDENTITY.with_rotation(base));
                let rotation = Quat::from_xyzw(
                    aligned.rotation.x as f32,
                    aligned.rotation.y as f32,
                    aligned.rotation.z as f32,
                    aligned.rotation.w as f32,
                );
                let (yaw_ros, _, _) = rotation.to_euler(EulerRot::ZYX);
                assert!(
                    (yaw_ros - yaw_deg.to_radians()).abs() < 1e-4,
                    "yaw {yaw_deg} deg with tilt {tilt_deg} mapped to {} rad",
                    yaw_ros.to_degrees()
                );
            }
        }
    }

    #[test]
    fn body_yaw_rate_combines_root_spin_and_kinematic_yaw() {
        // Upright root: the two yaw rates simply add in the body frame.
        let rate = body_yaw_rate(
            Quat::IDENTITY,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.5, 0.0),
            2.0,
        );
        assert!((rate - 2.5).abs() < 1e-5);

        // The kinematic yaw always spins about the body's own up axis: a root
        // pitched 90° about x turns the world-frame axis to world z, but in
        // the (co-rotating) body frame it is still the full rate on body z.
        let root = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
        let rate = body_yaw_rate(root, root, Vec3::ZERO, 2.0);
        assert!((rate - 2.0).abs() < 1e-5);
    }

    #[test]
    fn imu_accelerometer_reads_plus_g_at_rest() {
        assert_eq!(
            imu_accel_with_gravity(Vec3::ZERO),
            Vec3::new(0.0, 0.0, IMU_GRAVITY_MPS2)
        );
    }
}
