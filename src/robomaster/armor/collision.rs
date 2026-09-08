use avian3d::prelude::{
    CollisionEventsEnabled, CollisionStart, LinearVelocity, PhysicsSchedule, PhysicsStepSystems,
};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::ecs::system::SystemParam;
use bevy::ecs::system::lifetimeless::Read;
use bevy::prelude::{
    ChildOf, Commands, Entity, GlobalTransform, On, Plugin, Query, Res, ResMut, Resource, Update,
    Vec3, With, Without, warn,
};
use std::collections::HashSet;
use std::sync::Once;

use super::construct::Armor;
use super::incidence::{ArmorFrame, ArmorHitAngles, PreSolveVelocity};
use crate::config::SimulationConfig;
use crate::robomaster::power_rune::prelude::Projectile;
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
    armors: Query<'w, 's, Read<Armor>>,
    frames: Query<'w, 's, (&'static GlobalTransform, Option<&'static ArmorFrame>)>,
    child_of: Query<'w, 's, Read<ChildOf>>,
}

impl ArmorHitContext<'_, '_> {
    fn is_armor(&self, collider: Entity) -> bool {
        self.armors.contains(collider)
            || self
                .child_of
                .iter_ancestors(collider)
                .any(|ancestor| self.armors.contains(ancestor))
    }

    /// Applies the angular gate to a collider using the plate frame found on the collider
    /// or its nearest armor ancestor. Colliders without a derived frame count unfiltered.
    fn within_hit_zone(&self, collider: Entity, angles: &ArmorHitAngles, incoming: Vec3) -> bool {
        let mut entity = Some(collider);
        while let Some(current) = entity {
            if let Ok((transform, Some(frame))) = self.frames.get(current) {
                return angles.accepts(&frame.rotated_by(transform.rotation()), incoming);
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

/// Counts a projectile touching an armor plate, but only when the incoming direction lies
/// inside the rule's per-edge angular zone; contacts with the rear of a plate (e.g. a ball
/// that ghosted through the chassis) do not count.
fn handle_armor_collision(
    event: On<CollisionStart>,
    mut commands: Commands,
    config: Res<SimulationConfig>,
    mut stats: ResMut<ProjectileStatistics>,
    mut consumed: ResMut<ConsumedArmorProjectiles>,
    projectiles: Query<(Entity, &PreSolveVelocity), With<Projectile>>,
    context: ArmorHitContext,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());

    let (projectile_entity, incoming) = match (projectile_body1, projectile_body2) {
        (Some((entity, velocity)), _) | (_, Some((entity, velocity))) => (entity, velocity.0),
        _ => return,
    };

    let other_collider = if projectile_body1.is_some() {
        event.collider2
    } else {
        event.collider1
    };

    if !context.is_armor(other_collider) {
        return;
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
    if !context.within_hit_zone(other_collider, &angles, incoming) {
        // Out of zone: this contact does not count, but the projectile may still hit
        // another plate legitimately later.
        consumed.0.remove(&projectile_entity);
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
