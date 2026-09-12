//! Hit-incidence gating for armor plates.
//!
//! The judge system registers hits through the armor's force sensors, which respond to
//! the normal component of the contact impulse: a projectile grazing along the plate
//! face delivers almost no normal force and does not register. The gate therefore
//! bounds the angle between the incoming direction and the plate's outward normal.
//!
//! The RoboMaster construction-spec figures 105°/120°/145° are occlusion keep-out
//! zones around the plate edges ("受击打面下边缘 105°内不得被遮挡" restricts the robot's
//! own structure, not the projectile's arrival direction) and are deliberately NOT
//! used as incidence limits here.

use bevy::math::Vec2;
use bevy::prelude::{Component, Deref, DerefMut, Quat, Vec3};

/// Projectile velocity as it was before this physics tick's contact solving.
///
/// Collision events are triggered after the solver (`PhysicsStepSystems::Finalize`), so a
/// `LinearVelocity` read inside a collision observer is already post-bounce; this cache
/// preserves the incoming direction.
#[derive(Component, Debug, Deref, DerefMut)]
pub struct PreSolveVelocity(pub Vec3);

/// Outward-facing basis of an armor plate's hit face, expressed in the local space of the
/// entity that carries the armor collider. Rotate with [`Self::rotated_by`] using that
/// entity's current world rotation at hit time.
#[derive(Component, Debug, Clone)]
pub struct ArmorFrame {
    /// Outward normal of the hit face, pointing away from the owning robot.
    pub normal: Vec3,
    /// In-plane axis pointing toward the top edge of the plate.
    pub up: Vec3,
    /// In-plane axis toward the right edge (`up × normal`).
    pub right: Vec3,
}

impl ArmorFrame {
    pub fn rotated_by(&self, rotation: Quat) -> Self {
        Self {
            normal: rotation * self.normal,
            up: rotation * self.up,
            right: rotation * self.right,
        }
    }
}

/// Extent of the plate's hit face along the [`ArmorFrame`] right/up axes, measured from the
/// frame-holding entity's origin at construction time. Only contacts whose surface point
/// projects inside this rectangle count as hits on the face; anything else (module frame,
/// rim, ...) is spent without counting.
///
/// The scalars are invariant under the world→local rotation, so they are computed in world
/// space at construction and reused against [`ArmorFrame::rotated_by`] axes at hit time.
#[derive(Component, Debug, Clone, Copy)]
pub struct ArmorFaceBounds {
    /// Rectangle midpoint, `(along right, along up)`.
    pub center: Vec2,
    /// Half extents, `(along right, along up)`.
    pub half: Vec2,
}

impl ArmorFaceBounds {
    /// Whether `point` (projected onto the face axes) lies inside the face rectangle grown
    /// by `margin`, which absorbs the projectile radius and contact slop.
    pub fn contains(&self, point: Vec2, margin: f32) -> bool {
        let offset = (point - self.center).abs();
        offset.x <= self.half.x + margin && offset.y <= self.half.y + margin
    }
}

/// Maximum angle in degrees between the incoming projectile direction and the plate's
/// outward normal for a contact to count as a hit. The default of 75° matches the
/// incidence the construction spec uses when testing armor modules; arrivals beyond it
/// (grazing along the face, from behind, ...) are spent without counting. Values of
/// 180 or above accept every direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmorHitAngle {
    pub max: f32,
}

impl Default for ArmorHitAngle {
    fn default() -> Self {
        Self { max: 75.0 }
    }
}

impl ArmorHitAngle {
    /// Returns `true` when a projectile arriving with `incoming_velocity` hits a plate
    /// whose world-space basis is `frame` from within `max` degrees of the face normal.
    pub fn accepts(&self, frame: &ArmorFrame, incoming_velocity: Vec3) -> bool {
        // Direction from the plate toward the shooter.
        let Some(incoming) = (-incoming_velocity).try_normalize() else {
            return true; // Degenerate (resting) contact: keep counting the touch.
        };
        if self.max >= 180.0 {
            return true;
        }
        incoming.dot(frame.normal) >= self.max.to_radians().cos()
    }
}

/// Derives a world-space plate basis from construction-time geometry.
///
/// The GLB armor nodes use different local axis conventions across vehicle/HERO/outpost
/// exports, so the basis is derived geometrically instead:
/// - the marker quad fixes the face plane's normal line (any corner order, must be planar),
/// - the two vertex-cluster centroids fix the in-plane width axis,
/// - the plate owner's center fixes the outward sign (armor always faces away from it),
/// - world up fixes which in-plane way is "top" (plates stand near vertically).
pub fn derive_armor_frame(
    marker_points: &[Vec3; 4],
    vertex_centroids: [Vec3; 2],
    owner_center: Vec3,
) -> Option<ArmorFrame> {
    let face_center =
        (marker_points[0] + marker_points[1] + marker_points[2] + marker_points[3]) / 4.0;
    let normal_line = (marker_points[1] - marker_points[0])
        .cross(marker_points[2] - marker_points[0])
        .try_normalize()?;
    let width = (vertex_centroids[1] - vertex_centroids[0]).try_normalize()?;
    let up_line = normal_line.cross(width).try_normalize()?;

    let outward = (face_center - owner_center).dot(normal_line);
    if outward.abs() < 1e-6 {
        return None; // Owner center lies in the plate plane: outward sign is ambiguous.
    }
    let normal = normal_line * outward.signum();

    let toward_world_up = up_line.dot(Vec3::Y);
    if toward_world_up.abs() < 0.1 {
        return None; // Near-horizontal plate: which edge is "top" is ambiguous.
    }
    let up = up_line * toward_world_up.signum();
    let right = up.cross(normal).normalize_or_zero();
    (right != Vec3::ZERO).then_some(ArmorFrame { normal, up, right })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Plate facing +Z with world-up +Y, as seen by a shooter standing in front of it.
    fn frontal_frame() -> ArmorFrame {
        ArmorFrame {
            normal: Vec3::Z,
            up: Vec3::Y,
            right: Vec3::X,
        }
    }

    /// Incoming velocity for a shooter located in direction `dir` (plate -> shooter).
    fn incoming_from(dir: Vec3) -> Vec3 {
        -dir.normalize() * 25.0
    }

    fn close(actual: Vec3, expected: Vec3) -> bool {
        (actual - expected).length() < 1e-5
    }

    /// Shooter direction `deg_from_normal` away from the plate normal (+Z), rotated in
    /// the plate plane toward `tilt` (unit, e.g. ±Y/±X). 0° is head-on, 90° lies in the
    /// plate plane, 180° is straight behind the plate.
    fn front_dir(deg_from_normal: f32, tilt: Vec3) -> Vec3 {
        let a = deg_from_normal.to_radians();
        (Vec3::Z * a.cos() + tilt * a.sin()).normalize()
    }

    #[test]
    fn frontal_and_moderately_tilted_hits_count() {
        let angle = ArmorHitAngle::default();
        let frame = frontal_frame();
        // Head-on and at 45° toward each edge family.
        assert!(angle.accepts(&frame, incoming_from(Vec3::Z)));
        assert!(angle.accepts(&frame, incoming_from(front_dir(45.0, Vec3::Y))));
        assert!(angle.accepts(&frame, incoming_from(front_dir(45.0, Vec3::X))));
        // Just inside the limit still counts (74.9 rather than 75.0: the boundary
        // comparison is `>=`, and f32 normalization of the test direction would make an
        // exact-limit assertion knife-edge).
        assert!(angle.accepts(&frame, incoming_from(front_dir(74.9, Vec3::Y))));
    }

    /// The reported bug: shots arriving almost parallel to the plate face used to count,
    /// because the four per-edge half-space zones unioned into "anything but the rear" -
    /// every in-plane direction satisfied at least one of them.
    #[test]
    fn nearly_parallel_incidence_is_rejected_in_every_direction() {
        let angle = ArmorHitAngle::default();
        let frame = frontal_frame();
        for tilt in [Vec3::Y, Vec3::NEG_Y, Vec3::X, Vec3::NEG_X] {
            // Exactly in the plate plane (90°) and just past the limit (80°).
            assert!(
                !angle.accepts(&frame, incoming_from(front_dir(90.0, tilt))),
                "in-plane {tilt:?}"
            );
            assert!(
                !angle.accepts(&frame, incoming_from(front_dir(80.0, tilt))),
                "80 deg {tilt:?}"
            );
            // Just inside the limit: must still count.
            assert!(
                angle.accepts(&frame, incoming_from(front_dir(70.0, tilt))),
                "70 deg {tilt:?}"
            );
        }
    }

    #[test]
    fn behind_hits_rejected() {
        let angle = ArmorHitAngle::default();
        let frame = frontal_frame();
        // Straight behind, and just past the plate plane toward the rear.
        assert!(!angle.accepts(&frame, incoming_from(Vec3::NEG_Z)));
        assert!(!angle.accepts(&frame, incoming_from(front_dir(100.0, Vec3::NEG_Y))));
    }

    #[test]
    fn limit_is_tunable() {
        let strict = ArmorHitAngle { max: 60.0 };
        let frame = frontal_frame();
        assert!(strict.accepts(&frame, incoming_from(front_dir(55.0, Vec3::Y))));
        assert!(!strict.accepts(&frame, incoming_from(front_dir(65.0, Vec3::Y))));
    }

    #[test]
    fn degenerate_velocity_still_counts() {
        let angle = ArmorHitAngle::default();
        let frame = frontal_frame();
        assert!(angle.accepts(&frame, Vec3::ZERO));
    }

    #[test]
    fn limit_of_180_disables_the_check() {
        let angle = ArmorHitAngle { max: 180.0 };
        assert!(angle.accepts(&frontal_frame(), incoming_from(Vec3::NEG_Z)));
    }

    #[test]
    fn face_bounds_containment_respects_margin() {
        let bounds = ArmorFaceBounds {
            center: Vec2::new(0.01, -0.02),
            half: Vec2::new(0.115, 0.062),
        };
        let point = |right: f32, up: f32| Vec2::new(right, up);
        assert!(bounds.contains(point(0.01, -0.02), 0.0));
        assert!(bounds.contains(point(0.12, -0.08), 0.0)); // exact corner
        assert!(!bounds.contains(point(0.13, -0.02), 0.0));
        // The margin absorbs the projectile radius at the rim.
        assert!(bounds.contains(point(0.13, -0.02), 0.013));
        assert!(!bounds.contains(point(0.14, -0.02), 0.013));
    }

    #[test]
    fn frame_rotation_follows_plate() {
        let frame = frontal_frame();
        let rotated = frame.rotated_by(Quat::from_rotation_y(core::f32::consts::FRAC_PI_2));
        assert!(close(rotated.normal, Vec3::X));
        assert!(close(rotated.up, Vec3::Y));
    }

    /// vehicle.glb-style layout: plate in the z=0 plane, width along X, height along Y,
    /// owner (chassis) behind the plate at +Z, so the hit face looks toward -Z.
    #[test]
    fn derive_vehicle_style_layout() {
        let marker = [
            Vec3::new(-0.067, 0.026, 0.0),
            Vec3::new(-0.067, -0.026, 0.0),
            Vec3::new(0.067, 0.026, 0.0),
            Vec3::new(0.067, -0.026, 0.0),
        ];
        let vertices = [Vec3::new(-0.066, 0.0, 0.003), Vec3::new(0.066, 0.0, 0.003)];
        let frame = derive_armor_frame(&marker, vertices, Vec3::new(0.0, 0.0, 0.22))
            .expect("vehicle layout should derive");
        assert!(close(frame.normal, Vec3::NEG_Z));
        assert!(close(frame.up, Vec3::Y));
        assert!(close(frame.right, Vec3::NEG_X));
    }

    /// outpost.glb D/E/F-style layout: local axes permuted, plate in the x=0 plane with
    /// width along Z, height along Y, owner at +X.
    #[test]
    fn derive_outpost_style_permuted_layout() {
        let marker = [
            Vec3::new(0.0, 0.026, -0.067),
            Vec3::new(0.0, -0.026, -0.067),
            Vec3::new(0.0, 0.026, 0.067),
            Vec3::new(0.0, -0.026, 0.067),
        ];
        let vertices = [Vec3::new(0.003, 0.0, -0.066), Vec3::new(0.003, 0.0, 0.066)];
        let frame = derive_armor_frame(&marker, vertices, Vec3::new(0.2, 0.0, 0.0))
            .expect("outpost layout should derive");
        assert!(close(frame.normal, Vec3::NEG_X));
        assert!(close(frame.up, Vec3::Y));
    }

    /// outpost.glb E-style layout: the whole armor assembly tilted 29° around world up.
    #[test]
    fn derive_tilted_layout() {
        let tilt = Quat::from_rotation_y(0.5);
        let marker = [
            Vec3::new(-0.067, 0.026, 0.0),
            Vec3::new(-0.067, -0.026, 0.0),
            Vec3::new(0.067, 0.026, 0.0),
            Vec3::new(0.067, -0.026, 0.0),
        ]
        .map(|p| tilt * p);
        let vertices =
            [Vec3::new(-0.066, 0.0, 0.003), Vec3::new(0.066, 0.0, 0.003)].map(|p| tilt * p);
        let owner = tilt * Vec3::new(0.0, 0.0, 0.22);
        let frame =
            derive_armor_frame(&marker, vertices, owner).expect("tilted layout should derive");
        assert!(close(frame.normal, tilt * Vec3::NEG_Z));
        assert!(close(frame.up, Vec3::Y));
    }

    #[test]
    fn derive_rejects_degenerate_input() {
        // Collinear marker points: no plane normal.
        let collinear = [Vec3::ZERO, Vec3::X * 0.1, Vec3::X * 0.2, Vec3::X * 0.3];
        assert!(derive_armor_frame(&collinear, [Vec3::NEG_X, Vec3::X], Vec3::Y).is_none());

        // Owner center in the plate plane: outward sign ambiguous.
        let marker = [
            Vec3::new(-0.067, 0.026, 0.0),
            Vec3::new(-0.067, -0.026, 0.0),
            Vec3::new(0.067, 0.026, 0.0),
            Vec3::new(0.067, -0.026, 0.0),
        ];
        let vertices = [Vec3::new(-0.066, 0.0, 0.0), Vec3::new(0.066, 0.0, 0.0)];
        assert!(derive_armor_frame(&marker, vertices, Vec3::new(0.1, 0.0, 0.0)).is_none());

        // Horizontal plate (top edge not identifiable): width along X, normal along Y.
        let flat_marker = [
            Vec3::new(-0.067, 0.0, 0.026),
            Vec3::new(-0.067, 0.0, -0.026),
            Vec3::new(0.067, 0.0, 0.026),
            Vec3::new(0.067, 0.0, -0.026),
        ];
        let flat_vertices = [Vec3::new(-0.066, 0.003, 0.0), Vec3::new(0.066, 0.003, 0.0)];
        assert!(
            derive_armor_frame(&flat_marker, flat_vertices, Vec3::new(0.0, 0.2, 0.0)).is_none()
        );
    }
}
