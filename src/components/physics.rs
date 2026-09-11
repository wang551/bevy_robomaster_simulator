use avian3d::prelude::*;
use bevy::prelude::*;
use std::collections::HashMap;

use crate::robomaster::prelude::Team;

/// Team of the robot that launched this projectile; hit counting only accepts
/// contacts with enemy armor.
#[derive(Component, Copy, Clone, Debug, Deref)]
pub struct ProjectileTeam(pub Team);

#[derive(PhysicsLayer, Default, Clone, Copy, Debug)]
pub enum GameLayer {
    #[default]
    Default,
    VehicleSelf,
    VehicleOther,
    ProjectileSelf,
    ProjectileOther,
    Environment,
}

impl GameLayer {
    pub fn environment_collision_layers() -> CollisionLayers {
        CollisionLayers::new(
            Self::Environment,
            [
                Self::Default,
                Self::VehicleSelf,
                Self::VehicleOther,
                Self::ProjectileSelf,
                Self::ProjectileOther,
            ],
        )
    }

    pub fn vehicle_body_collision_layers(is_self: bool) -> CollisionLayers {
        if is_self {
            CollisionLayers::new(
                Self::VehicleSelf,
                [Self::Default, Self::VehicleOther, Self::Environment],
            )
        } else {
            CollisionLayers::new(
                Self::VehicleOther,
                [
                    Self::Default,
                    Self::VehicleSelf,
                    Self::VehicleOther,
                    Self::Environment,
                ],
            )
        }
    }

    pub fn vehicle_armor_collision_layers(is_self: bool) -> CollisionLayers {
        if is_self {
            CollisionLayers::new(
                Self::VehicleSelf,
                [
                    Self::Default,
                    Self::VehicleOther,
                    Self::ProjectileOther,
                    Self::Environment,
                ],
            )
        } else {
            CollisionLayers::new(
                Self::VehicleOther,
                [
                    Self::Default,
                    Self::VehicleSelf,
                    Self::ProjectileSelf,
                    Self::Environment,
                ],
            )
        }
    }

    pub fn projectile_collision_layers(is_self: bool) -> CollisionLayers {
        if is_self {
            CollisionLayers::new(
                Self::ProjectileSelf,
                [Self::Default, Self::VehicleOther, Self::Environment],
            )
        } else {
            CollisionLayers::new(
                Self::ProjectileOther,
                [Self::Default, Self::VehicleSelf, Self::Environment],
            )
        }
    }

    /// Layers for colliders on a robot's non-armor visual meshes (chassis shell, gimbal
    /// tower). They exist only so projectiles are absorbed by the body instead of
    /// ghosting through it, so they interact with nothing but opposing projectiles:
    /// driving collisions stay on the root body cylinder.
    pub fn vehicle_obstacle_collision_layers(is_self: bool) -> CollisionLayers {
        if is_self {
            CollisionLayers::new(Self::VehicleSelf, [Self::ProjectileOther])
        } else {
            CollisionLayers::new(Self::VehicleOther, [Self::ProjectileSelf])
        }
    }
}

#[derive(Component, Deref, DerefMut)]
pub struct ProjectileLifetime(pub Timer);

#[derive(Resource, Deref, DerefMut)]
pub struct ProjectileCooldown(pub Timer);

#[derive(Resource)]
pub struct ProjectileSetting(pub Handle<Mesh>, pub Handle<StandardMaterial>);

#[derive(Resource)]
pub struct DartSetting(pub Handle<WorldAsset>);

#[derive(Component)]
pub struct GroundRoot;

#[derive(Component)]
pub struct DartLaunch;

#[derive(Component)]
pub struct DartProjectile;

#[derive(Component, Deref, DerefMut)]
pub struct PreciousCollision(
    pub  HashMap<
        String,
        (
            ColliderConstructorHierarchy,
            CollisionLayers,
            Visibility,
            Option<RigidBody>,
        ),
    >,
);

pub const PROJECTILE_LIFETIME_SECS: f32 = 5.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn self_projectile_ignores_self_vehicle_and_projectiles() {
        let projectile = GameLayer::projectile_collision_layers(true);

        assert!(!projectile.interacts_with(GameLayer::vehicle_body_collision_layers(true)));
        assert!(!projectile.interacts_with(GameLayer::vehicle_armor_collision_layers(true)));
        assert!(!projectile.interacts_with(GameLayer::projectile_collision_layers(true)));
        assert!(!projectile.interacts_with(GameLayer::projectile_collision_layers(false)));
    }

    #[test]
    fn projectiles_hit_opposing_armor_and_environment() {
        let self_projectile = GameLayer::projectile_collision_layers(true);
        let other_projectile = GameLayer::projectile_collision_layers(false);

        assert!(self_projectile.interacts_with(GameLayer::vehicle_armor_collision_layers(false)));
        assert!(other_projectile.interacts_with(GameLayer::vehicle_armor_collision_layers(true)));
        assert!(self_projectile.interacts_with(GameLayer::environment_collision_layers()));
        assert!(other_projectile.interacts_with(GameLayer::environment_collision_layers()));
    }

    #[test]
    fn projectiles_do_not_hit_vehicle_body_colliders() {
        let self_projectile = GameLayer::projectile_collision_layers(true);
        let other_projectile = GameLayer::projectile_collision_layers(false);

        assert!(!self_projectile.interacts_with(GameLayer::vehicle_body_collision_layers(false)));
        assert!(!other_projectile.interacts_with(GameLayer::vehicle_body_collision_layers(true)));
    }

    #[test]
    fn body_meshes_absorb_only_opposing_projectiles() {
        let self_obstacle = GameLayer::vehicle_obstacle_collision_layers(true);
        let other_obstacle = GameLayer::vehicle_obstacle_collision_layers(false);
        let self_projectile = GameLayer::projectile_collision_layers(true);
        let other_projectile = GameLayer::projectile_collision_layers(false);

        assert!(self_projectile.interacts_with(other_obstacle));
        assert!(other_projectile.interacts_with(self_obstacle));
        assert!(!self_projectile.interacts_with(self_obstacle));
        assert!(!other_projectile.interacts_with(other_obstacle));
        // Obstacles take no part in driving physics or armor contacts.
        assert!(!self_obstacle.interacts_with(other_obstacle));
        assert!(!self_obstacle.interacts_with(GameLayer::environment_collision_layers()));
        assert!(!self_obstacle.interacts_with(GameLayer::vehicle_body_collision_layers(false)));
        assert!(!self_obstacle.interacts_with(GameLayer::vehicle_armor_collision_layers(false)));
    }
}
