//! Hit-incidence gating for armor plates.
//!
//! RoboMaster rule: 装甲模块受击打面下边缘 105°内、上边缘 120°内、左右边缘 145°内不得被遮挡.
//! A projectile therefore only counts as a hit when its incoming direction lies inside that
//! zone: at most `bottom`/`top`/`side` degrees from the hit face around the corresponding
//! edge, i.e. at most 15°/30°/55° past the plate plane toward the rear of the plate.

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

/// Per-edge angular limits in degrees, measured from the hit face around the corresponding
/// edge, as in the rule text. Values of 180 or above disable an edge's limit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmorHitAngles {
    pub bottom: f32,
    pub top: f32,
    pub side: f32,
}

impl Default for ArmorHitAngles {
    fn default() -> Self {
        Self {
            bottom: 105.0,
            top: 120.0,
            side: 145.0,
        }
    }
}

impl ArmorHitAngles {
    /// Returns `true` when a projectile arriving with `incoming_velocity` hits a plate
    /// whose world-space basis is `frame` from within the rule's angular zone.
    pub fn accepts(&self, frame: &ArmorFrame, incoming_velocity: Vec3) -> bool {
        // Direction from the plate toward the shooter.
        let Some(incoming) = (-incoming_velocity).try_normalize() else {
            return true; // Degenerate (resting) contact: keep counting the touch.
        };
        let (sr, su, sn) = (
            incoming.dot(frame.right),
            incoming.dot(frame.up),
            incoming.dot(frame.normal),
        );
        // Each edge's zone is a half space bounded by a plane tilted `rule - 90°` past the
        // plate plane around that edge; the shooter direction must lie in at least one of
        // the four zones. Frontal directions satisfy all of them.
        let in_sector = |rule_deg: f32, toward_edge: f32| {
            if rule_deg >= 180.0 {
                return true;
            }
            let margin = (rule_deg - 90.0).to_radians();
            sn * margin.cos() + toward_edge * margin.sin() >= 0.0
        };
        in_sector(self.bottom, -su)
            || in_sector(self.top, su)
            || in_sector(self.side, sr)
            || in_sector(self.side, -sr)
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

    /// Shooter direction `past_plane_deg` past the plate plane toward the rear, rotated
    /// away from the normal toward in-plane direction `tilt` (unit, e.g. ±Y/±X).
    /// 0° lies in the plate plane, 90° is straight behind the plate.
    fn shooter_dir(past_plane_deg: f32, tilt: Vec3) -> Vec3 {
        let past = past_plane_deg.to_radians();
        (tilt * past.cos() - Vec3::Z * past.sin()).normalize()
    }

    #[test]
    fn frontal_and_grazing_hits_count() {
        let angles = ArmorHitAngles::default();
        let frame = frontal_frame();
        // Straight frontal.
        assert!(angles.accepts(&frame, incoming_from(Vec3::Z)));
        // Frontal hemisphere at 45° up and 80° down.
        assert!(angles.accepts(
            &frame,
            incoming_from((Vec3::Z * 1.0 + Vec3::Y * 1.0).normalize())
        ));
        assert!(angles.accepts(
            &frame,
            incoming_from((Vec3::Z + Vec3::NEG_Y * 5.7).normalize())
        ));
        // Grazing along the face (in-plane, 90° around an edge): inside every limit.
        assert!(angles.accepts(&frame, incoming_from(Vec3::Y)));
        assert!(angles.accepts(&frame, incoming_from(Vec3::NEG_Y)));
        assert!(angles.accepts(&frame, incoming_from(Vec3::X)));
    }

    #[test]
    fn behind_hits_rejected() {
        let angles = ArmorHitAngles::default();
        let frame = frontal_frame();
        assert!(!angles.accepts(&frame, incoming_from(Vec3::NEG_Z)));
    }

    #[test]
    fn bottom_edge_zone_is_105_degrees() {
        let angles = ArmorHitAngles::default();
        let frame = frontal_frame();
        // 10° below-behind: inside the 15° margin past the plane.
        assert!(angles.accepts(&frame, incoming_from(shooter_dir(10.0, Vec3::NEG_Y))));
        // 20° below-behind: outside.
        assert!(!angles.accepts(&frame, incoming_from(shooter_dir(20.0, Vec3::NEG_Y))));
    }

    #[test]
    fn top_edge_zone_is_120_degrees() {
        let angles = ArmorHitAngles::default();
        let frame = frontal_frame();
        assert!(angles.accepts(&frame, incoming_from(shooter_dir(25.0, Vec3::Y))));
        assert!(!angles.accepts(&frame, incoming_from(shooter_dir(35.0, Vec3::Y))));
    }

    #[test]
    fn side_edge_zone_is_145_degrees() {
        let angles = ArmorHitAngles::default();
        let frame = frontal_frame();
        assert!(angles.accepts(&frame, incoming_from(shooter_dir(50.0, Vec3::X))));
        assert!(!angles.accepts(&frame, incoming_from(shooter_dir(60.0, Vec3::X))));
        // Symmetric on the other side.
        assert!(angles.accepts(&frame, incoming_from(shooter_dir(50.0, Vec3::NEG_X))));
        assert!(!angles.accepts(&frame, incoming_from(shooter_dir(60.0, Vec3::NEG_X))));
    }

    #[test]
    fn degenerate_velocity_still_counts() {
        let angles = ArmorHitAngles::default();
        let frame = frontal_frame();
        assert!(angles.accepts(&frame, Vec3::ZERO));
    }

    #[test]
    fn limits_of_180_disable_the_check() {
        let angles = ArmorHitAngles {
            bottom: 180.0,
            top: 180.0,
            side: 180.0,
        };
        assert!(angles.accepts(&frontal_frame(), incoming_from(Vec3::NEG_Z)));
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
