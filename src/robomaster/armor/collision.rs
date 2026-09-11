use avian3d::prelude::{
    CollisionEventsEnabled, CollisionStart, LinearVelocity, PhysicsSchedule, PhysicsStepSystems,
    Position,
};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::ecs::system::SystemParam;
use bevy::ecs::system::lifetimeless::Read;
use bevy::prelude::{
    ChildOf, Commands, Entity, GlobalTransform, On, Plugin, Query, Res, ResMut, Resource, Update,
    Vec2, Vec3, With, Without, warn,
};
use std::collections::HashSet;
use std::sync::Once;

use super::construct::Armor;
use super::incidence::{ArmorFaceBounds, ArmorFrame, ArmorHitAngles, PreSolveVelocity};
use crate::components::Infantry;
use crate::components::ProjectileTeam;
use crate::config::SimulationConfig;
use crate::robomaster::power_rune::prelude::Projectile;
use crate::robomaster::prelude::Team;
use crate::statistic::ProjectileStatistics;

/// Avian triggers collision events after contact solving (`PhysicsStepSystems::Finalize`),
/// so a projectile's `LinearVelocity` seen by a collision observer is already post-bounce.
/// This cache snapshots the pre-solve velocity every physics tick for incidence checks.
fn cache_projectile_pre_solve_velocity(
    mut commands: Commands,
    uncached: Query<(Entity, &LinearVelocity), (With<Projectile>, Without<PreSolveVelocity>)>,
    mut cached: Query<(&LinearVelocity, &mut PreSolveVelocity), With<Projectile>>,
) {
    for (entity, velocity) in &uncached {
        commands.entity(entity).insert(PreSolveVelocity(velocity.0));
    }
    for (velocity, mut pre_solve) in &mut cached {
        pre_solve.0 = velocity.0;
    }
}

#[derive(Resource, Default)]
struct ConsumedArmorProjectiles(HashSet<Entity>);

fn cleanup_consumed_armor_projectiles(
    mut consumed: ResMut<ConsumedArmorProjectiles>,
    projectiles: Query<(), With<Projectile>>,
) {
    consumed.0.retain(|entity| projectiles.contains(*entity));
}

#[derive(SystemParam)]
struct ArmorHitContext<'w, 's> {
    armors: Query<'w, 's, &'static Armor>,
    infantry: Query<'w, 's, &'static Infantry>,
    frames: Query<
        'w,
        's,
        (
            &'static GlobalTransform,
            Option<&'static ArmorFrame>,
            Option<&'static ArmorFaceBounds>,
        ),
    >,
    child_of: Query<'w, 's, Read<ChildOf>>,
}

impl ArmorHitContext<'_, '_> {
    /// True when the collider belongs to a robot's body (gimbal structure, chassis
    /// mesh, ...) rather than an armor plate or the environment.
    fn is_robot_body(&self, collider: Entity) -> bool {
        let mut entity = Some(collider);
        while let Some(current) = entity {
            if self.infantry.contains(current) {
                return true;
            }
            entity = self
                .child_of
                .get(current)
                .ok()
                .map(|child_of| child_of.parent());
        }
        false
    }

    /// Team of the armor plate the collider belongs to, if it is one.
    fn armor_team(&self, collider: Entity) -> Option<Team> {
        let mut entity = Some(collider);
        while let Some(current) = entity {
            if let Ok(armor) = self.armors.get(current) {
                return Some(armor.team);
            }
            entity = self
                .child_of
                .get(current)
                .ok()
                .map(|child_of| child_of.parent());
        }
        None
    }

    /// Applies the face-rectangle and angular gates to a collider using the plate frame
    /// found on the collider or its nearest armor ancestor. Colliders without a derived
    /// frame count unfiltered. A contact whose surface point projects outside the face
    /// rectangle (module rim, frame) is not a face hit.
    fn within_hit_zone(
        &self,
        collider: Entity,
        angles: &ArmorHitAngles,
        incoming: Vec3,
        ball_position: Option<Vec3>,
        face_margin: f32,
    ) -> bool {
        let mut entity = Some(collider);
        while let Some(current) = entity {
            if let Ok((transform, Some(frame), bounds)) = self.frames.get(current) {
                let world = frame.rotated_by(transform.rotation());
                if let (Some(bounds), Some(ball)) = (bounds, ball_position) {
                    let relative = ball - transform.translation();
                    let point = Vec2::new(relative.dot(world.right), relative.dot(world.up));
                    if !bounds.contains(point, face_margin) {
                        return false;
                    }
                }
                return angles.accepts(&world, incoming);
            }
            entity = self
                .child_of
                .get(current)
                .ok()
                .map(|child_of| child_of.parent());
        }

        static UNFRAMED_ARMOR_WARNED: Once = Once::new();
        UNFRAMED_ARMOR_WARNED.call_once(|| {
            warn!("armor collider without a derived ArmorFrame: its hits count unfiltered");
        });
        true
    }
}

/// Counts a projectile touching an enemy armor plate, but only when the incoming
/// direction lies inside the rule's per-edge angular zone. The first decisive contact
/// determines the projectile's fate:
/// - enemy armor in zone: counts;
/// - enemy armor out of zone: spent without counting (otherwise a ball ghosting through
///   the chassis would register on whatever armor happens to be behind it);
/// - a robot's non-armor body (gimbal structure, chassis mesh): spent as well, so a
///   ricochet off the body onto a plate afterwards does not register;
/// - friendly armor and the environment are ignored and never consume the projectile.
fn handle_armor_collision(
    event: On<CollisionStart>,
    mut commands: Commands,
    config: Res<SimulationConfig>,
    mut stats: ResMut<ProjectileStatistics>,
    mut consumed: ResMut<ConsumedArmorProjectiles>,
    projectiles: Query<
        (
            Entity,
            &PreSolveVelocity,
            &ProjectileTeam,
            Option<&Position>,
        ),
        With<Projectile>,
    >,
    context: ArmorHitContext,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());

    let (projectile_entity, incoming, ball_team, ball_position) =
        match (projectile_body1, projectile_body2) {
            (Some((entity, velocity, team, position)), _)
            | (_, Some((entity, velocity, team, position))) => {
                (entity, velocity.0, team.0, position.map(|p| p.0))
            }
            _ => return,
        };

    let other_collider = if projectile_body1.is_some() {
        event.collider2
    } else {
        event.collider1
    };

    match context.armor_team(other_collider) {
        None => {
            if context.is_robot_body(other_collider) {
                consumed.0.insert(projectile_entity);
            }
            return;
        }
        Some(team) if team == ball_team => return,
        Some(_) => {}
    }

    // Observer commands are deferred, so two plates touched within one physics tick would
    // both count; reserve the projectile immediately instead.
    if !consumed.0.insert(projectile_entity) {
        return;
    }

    let angles = ArmorHitAngles {
        bottom: config.armor.hit_angle_bottom,
        top: config.armor.hit_angle_top,
        side: config.armor.hit_angle_side,
    };
    // The margin absorbs the projectile radius plus contact slop, since avian reports the
    // solved ball center rather than the surface contact point.
    let face_margin = config.projectile.diameter * 0.5 + 0.005;
    if !context.within_hit_zone(
        other_collider,
        &angles,
        incoming,
        ball_position,
        face_margin,
    ) {
        return;
    }

    // Disable collision events for this projectile so it only counts once
    commands
        .entity(projectile_entity)
        .remove::<CollisionEventsEnabled>();
    stats.increase_accurate();
}

#[derive(Default)]
pub(super) struct ArmorCollisionPlugin;

impl Plugin for ArmorCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<ConsumedArmorProjectiles>()
            .add_systems(
                PhysicsSchedule,
                cache_projectile_pre_solve_velocity.in_set(PhysicsStepSystems::First),
            )
            .add_systems(Update, cleanup_consumed_armor_projectiles)
            .add_observer(handle_armor_collision);
    }
}
