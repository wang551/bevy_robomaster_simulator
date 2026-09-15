//! Scene load order.
//!
//! glTF worlds become ready on whatever frame their asset happens to finish loading, so spawning
//! everything at once lets `setup_vehicle` turn a robot into a dynamic rigid body before the
//! environment has any colliders — and the robot falls through the floor. Loading is therefore one
//! linear task: each `spawn` resolves when that world's instance is ready, and the robots are
//! spawned last.
//!
//! Per-world setup (naming, collision layers, gimbal wiring) stays in the `WorldInstanceReady`
//! observers, because worlds spawned at runtime — projectiles, UAVs — need it too. What used to be
//! implicit ordering *between* those observers is now explicit here.

use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::world_serialization::{InstanceId, WorldAssetRoot, WorldInstance};
use std::collections::HashMap;

use crate::components::{
    ActiveSlapper, Controlled, GameLayer, GroundRoot, Infantry, PreciousCollision, SlapperInfantry,
};
use crate::config::{MapKind, SimulationConfig};
use crate::robomaster::power_rune::construct::setup_power_rune;
use crate::robomaster::prelude::{
    HERO_ROBOT_CONFIG, INFANTRY_THREE_CONFIG, PowerRuneRoot, Team, TechCoreRoot,
};
use crate::robomaster::tech_core::construct::setup_tech_core;
use crate::setup::{
    ScanOutpost, setup_collision, setup_dart_launch, setup_outposts, setup_vehicle,
};
use crate::util::async_world::{AsyncWorld, AsyncWorldTask, drive_async_world};

/// Resolves once Avian has built every collider `setup_collision` queued under `root`.
///
/// `setup_collision` puts a [`ColliderConstructorHierarchy`] on the named descendants of a world as
/// soon as its instance is ready, so by the time this runs the queue for `root` is complete.
/// Selecting and observing happen in one world job, so constructors finishing early are simply not
/// selected rather than awaited forever.
async fn colliders_ready(w: &AsyncWorld, root: Entity) {
    w.observe_all::<ColliderConstructorHierarchyReady, _>(move |world| {
        let mut queued = Vec::new();
        collect_descendants(world, root, &mut queued);
        queued.retain(|entity| world.get::<ColliderConstructorHierarchy>(*entity).is_some());
        queued
    })
    .await;
}

/// The instance id a loaded world spawned under, needed to enumerate its entities by name.
async fn instance_of(w: &AsyncWorld, root: Entity) -> InstanceId {
    w.with_world(move |world| {
        **world
            .get::<WorldInstance>(root)
            .expect("world instance is ready, so its id must exist")
    })
    .await
}

fn collect_descendants(world: &World, entity: Entity, out: &mut Vec<Entity>) {
    let Some(children) = world.get::<Children>(entity) else {
        return;
    };
    let children: Vec<Entity> = children.iter().collect();
    for child in children {
        out.push(child);
        collect_descendants(world, child, out);
    }
}

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(AsyncWorldTask::new(load_scene))
            .add_systems(
                PreUpdate,
                drive_async_world.run_if(resource_exists::<AsyncWorldTask>),
            );
    }
}

async fn load_scene(w: AsyncWorld) {
    let assets = w
        .with_world(|world| world.resource::<AssetServer>().clone())
        .await;
    let scene =
        |path: &'static str| WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(path)));

    let layers = GameLayer::environment_collision_layers();
    let trimesh = || {
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
            TrimeshFlags::all(),
        ))
        .with_default_layers(layers)
    };
    let static_trimesh = || {
        (
            trimesh(),
            layers,
            Visibility::Visible,
            Some(RigidBody::Static),
        )
    };

    // The map selection is read once at startup; scene assets are not hot-reloaded.
    let map = w
        .with_world(|world| world.resource::<SimulationConfig>().scene.map)
        .await;
    match map {
        MapKind::Rmuc => load_rmuc_scene(&w, &scene, &static_trimesh, layers).await,
        MapKind::Rmul => load_rmul_scene(&w, &scene, &static_trimesh).await,
    }

    info!("scene loaded");
}

/// What a `setup_collision` entry looks like for the RMUC map.
type CollisionEntry = (
    ColliderConstructorHierarchy,
    CollisionLayers,
    Visibility,
    Option<RigidBody>,
);

/// Full RMUC arena: ground + outposts + power rune + tech core + robots.
async fn load_rmuc_scene(
    w: &AsyncWorld,
    scene: &(impl Fn(&'static str) -> WorldAssetRoot + Sync),
    static_trimesh: &(impl Fn() -> CollisionEntry + Sync),
    layers: CollisionLayers,
) {
    // Environment first. Every robot below lands on what these build.
    let ground = w
        .spawn(scene("GROUND.glb"), (GroundRoot, Friction::new(0.5)))
        .await;
    w.run(setup_dart_launch, ground).await;
    w.run(
        setup_collision,
        (
            ground,
            PreciousCollision(HashMap::from([(
                "GROUND_DENSE".to_string(),
                static_trimesh(),
            )])),
        ),
    )
    .await;
    colliders_ready(w, ground).await;

    spawn_calib_boards(w, scene).await;

    let outpost = w
        .spawn(scene("OUTPOST.glb"), (RigidBody::Static, ScanOutpost))
        .await;
    w.run(setup_outposts, outpost).await;

    let tech_core = w.spawn(scene("TECH_CORE.glb"), TechCoreRoot).await;
    w.run(
        setup_tech_core,
        (tech_core, instance_of(w, tech_core).await),
    )
    .await;
    w.run(
        setup_collision,
        (
            tech_core,
            PreciousCollision(HashMap::from([("GROUND".to_string(), static_trimesh())])),
        ),
    )
    .await;
    colliders_ready(w, tech_core).await;

    let power_rune = w
        .spawn(
            scene("POWER.glb"),
            (
                RigidBody::Static,
                CollisionMargin(0.001),
                Restitution::ZERO,
                PowerRuneRoot,
            ),
        )
        .await;
    w.run(
        setup_power_rune,
        (power_rune, instance_of(w, power_rune).await),
    )
    .await;
    w.run(
        setup_collision,
        (power_rune, PreciousCollision(power_rune_collision(layers))),
    )
    .await;
    colliders_ready(w, power_rune).await;

    spawn_robots(w, scene).await;
}

/// Simple RMUL field: one self-contained `GROUND_RMUL.glb`, no outposts /
/// power rune / tech core. `SHELL` is the wall-top cap ring; `SOLID` carries the
/// drivable floor, walls, plateaus and ramps. Both get trimesh colliders so
/// projectiles and vehicles hit the field instead of ghosting through.
///
/// The asset is a raw CAD export whose SOLID mesh contained 2-13mm construction
/// plates (central pad, corner plates, wall-base lip) above the floor. Vehicles
/// are flat-bottomed cylinders with a 5mm collision margin, so those steps were
/// impassable walls; `tools/flatten_rmul_floor.py` compresses them into the
/// 1.0-1.4mm range (monotonically, so overlapping surfaces like the 13mm seam
/// above the 12mm pad never land on a shared plane and z-fight). Re-run it
/// whenever the field model is re-exported (regression test in [`glb_assets`]).
async fn load_rmul_scene(
    w: &AsyncWorld,
    scene: &(impl Fn(&'static str) -> WorldAssetRoot + Sync),
    static_trimesh: &(impl Fn() -> CollisionEntry + Sync),
) {
    let ground = w
        .spawn(scene("GROUND_RMUL.glb"), (GroundRoot, Friction::new(0.5)))
        .await;
    w.run(
        setup_collision,
        (
            ground,
            PreciousCollision(HashMap::from([
                ("SHELL".to_string(), static_trimesh()),
                ("SOLID".to_string(), static_trimesh()),
            ])),
        ),
    )
    .await;
    colliders_ready(w, ground).await;

    spawn_calib_boards(w, scene).await;

    spawn_robots(w, scene).await;
}

/// Debug calibration boards, shared by every map.
async fn spawn_calib_boards(
    w: &AsyncWorld,
    scene: &(impl Fn(&'static str) -> WorldAssetRoot + Sync),
) {
    w.spawn(
        scene("CALIB.glb"),
        Transform::IDENTITY.with_translation(Vec3::new(1.0, 2.5, 1.0)),
    )
    .await;

    w.spawn(
        scene("CALIB.glb"),
        Transform::IDENTITY.with_translation(Vec3::new(2.0, 0.5, 2.0)),
    )
    .await;
}

/// Robots land last so they can never become dynamic over an empty world.
async fn spawn_robots(w: &AsyncWorld, scene: &(impl Fn(&'static str) -> WorldAssetRoot + Sync)) {
    let player = w
        .spawn(
            scene("vehicle.glb"),
            (
                Transform::from_xyz(0.0, 1.0, 0.0),
                Infantry::new(Team::Red, INFANTRY_THREE_CONFIG),
                Controlled,
            ),
        )
        .await;
    w.run(setup_vehicle, player).await;

    let slapper = w
        .spawn(
            scene("vehicle.glb"),
            (
                Transform::from_xyz(1.0, 1.0, 1.0),
                Infantry::new(Team::Blue, INFANTRY_THREE_CONFIG),
                SlapperInfantry,
            ),
        )
        .await;
    w.run(setup_vehicle, slapper).await;

    let hero = w
        .spawn(
            scene("HERO.glb"),
            (
                Transform::from_xyz(2.0, 1.0, 1.0),
                Infantry::new(Team::Blue, HERO_ROBOT_CONFIG),
                SlapperInfantry,
                ActiveSlapper,
            ),
        )
        .await;
    w.run(setup_vehicle, hero).await;
}

/// Every rune target gets a voxelized collider so hits register on the spinning arms.
fn power_rune_collision(
    layers: CollisionLayers,
) -> HashMap<
    String,
    (
        ColliderConstructorHierarchy,
        CollisionLayers,
        Visibility,
        Option<RigidBody>,
    ),
> {
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
        .with_default_layers(layers)
    };

    let mut collision = HashMap::from([(
        "BASE".to_string(),
        (
            ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
                TrimeshFlags::all(),
            ))
            .with_default_layers(layers),
            layers,
            Visibility::Visible,
            Some(RigidBody::Static),
        ),
    )]);
    for face in 1..=2 {
        for target in 1..=5 {
            for state in ["ACTIVATED", "ACTIVE", "COMPLETED", "DISABLED"] {
                collision.insert(
                    format!("FACE_{face}_TARGET_{target}_{state}"),
                    (voxel(0.015), layers, Visibility::Visible, None),
                );
            }
        }
    }
    collision
}

/// The GLB assets are the only model source of truth (no .blend files are tracked), so
/// they are guarded here against exporting mistakes instead of at runtime.
#[cfg(test)]
mod glb_assets {
    use serde_json::Value;

    /// Parse the JSON chunk out of a GLB container.
    fn glb_json_chunk(path: &str) -> Value {
        let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("failed to read {path}: {e}"));
        assert!(bytes.starts_with(b"glTF"), "{path} is not a GLB container");
        let json_len = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
        assert_eq!(&bytes[16..20], b"JSON", "first chunk of {path} is not JSON");
        serde_json::from_slice(&bytes[20..20 + json_len]).expect("GLB JSON chunk should parse")
    }

    fn vehicle_glb() -> Value {
        glb_json_chunk(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/vehicle.glb"))
    }

    /// vehicle.glb was once exported with stray laser-detector assemblies (激光检测总装)
    /// parented under GIMBAL, rendering as floating "tech cores" above every vehicle.
    /// `tools/strip_vehicle_stray_nodes.py` removes them; this keeps them from coming back.
    #[test]
    fn vehicle_glb_has_no_stray_tech_core_nodes() {
        let json = vehicle_glb();
        let strays: Vec<&str> = json["nodes"]
            .as_array()
            .expect("nodes array")
            .iter()
            .filter_map(|node| node["name"].as_str())
            .filter(|name| name.contains("激光检测总装"))
            .collect();
        assert!(
            strays.is_empty(),
            "vehicle.glb regained stray nodes {strays:?}; re-run tools/strip_vehicle_stray_nodes.py"
        );
    }

    /// `setup_vehicle` resolves VEHICLE/GIMBAL/BASE by name at runtime, and the glTF loader
    /// walks the scene hierarchy — both must stay intact through any asset surgery.
    #[test]
    fn vehicle_glb_hierarchy_is_well_formed() {
        let json = vehicle_glb();
        let nodes = json["nodes"].as_array().expect("nodes array");
        for required in ["VEHICLE", "GIMBAL", "BASE"] {
            assert!(
                nodes.iter().any(|node| node["name"] == required),
                "vehicle.glb lost required node {required}"
            );
        }
        let index =
            |value: &Value| value.as_u64().expect("index is a non-negative integer") as usize;
        for scene in json["scenes"].as_array().into_iter().flatten() {
            for root in scene["nodes"].as_array().into_iter().flatten() {
                assert!(index(root) < nodes.len(), "scene root index out of range");
            }
        }
        let mesh_count = json["meshes"].as_array().map_or(0, |meshes| meshes.len());
        for node in nodes {
            for child in node["children"].as_array().into_iter().flatten() {
                assert!(index(child) < nodes.len(), "child index out of range");
            }
            if let Some(mesh) = node["mesh"].as_u64() {
                assert!((mesh as usize) < mesh_count, "mesh index out of range");
            }
        }
    }

    /// Read the BIN chunk out of a GLB container (JSON is always the first chunk).
    fn glb_bin_chunk(path: &str) -> Vec<u8> {
        let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("failed to read {path}: {e}"));
        let total = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
        let mut offset = 12;
        while offset + 8 <= total {
            let clen = u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap()) as usize;
            if &bytes[offset + 4..offset + 8] == b"BIN\x00" {
                return bytes[offset + 8..offset + 8 + clen].to_vec();
            }
            offset += 8 + clen;
        }
        panic!("{path} has no BIN chunk");
    }

    /// GROUND_RMUL.glb is a raw CAD export whose SOLID mesh carried 2-13mm
    /// construction plates (central pad, corner plates, wall-base lip) above the
    /// drivable floor. Vehicles are flat-bottomed cylinders with a 5mm collision
    /// margin, so those steps were impassable walls. `tools/flatten_rmul_floor.py`
    /// compresses the offending vertices into 1.0..1.4mm, monotonically so that
    /// overlapping surfaces (the 13mm seam above the 12mm pad) never share a
    /// plane and z-fight; this keeps a re-export from regressing.
    #[test]
    fn ground_rmul_glb_construction_plates_are_flattened() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/GROUND_RMUL.glb");
        let json = glb_json_chunk(path);
        let bin = glb_bin_chunk(path);

        // World Y = -local Y via the root's 180-degree rotation about (1, 0, -1)/sqrt(2).
        let root_idx = json["scenes"][0]["nodes"][0]
            .as_u64()
            .expect("scene root index") as usize;
        let rotation = json["nodes"][root_idx]["rotation"]
            .as_array()
            .expect("root rotation");
        let expected = [
            std::f32::consts::FRAC_1_SQRT_2,
            0.0,
            -std::f32::consts::FRAC_1_SQRT_2,
            0.0,
        ];
        for (component, expected) in rotation.iter().zip(expected) {
            let component = component.as_f64().expect("rotation component");
            assert!(
                (component - f64::from(expected)).abs() < 1e-3,
                "unexpected root rotation; vertex heights no longer map world Y = -local Y"
            );
        }

        let solid = json["nodes"]
            .as_array()
            .expect("nodes array")
            .iter()
            .find(|node| node["name"] == "SOLID")
            .expect("SOLID node");
        let solid_mesh = solid["mesh"].as_u64().expect("SOLID mesh index") as usize;
        let prims = json["meshes"][solid_mesh]["primitives"]
            .as_array()
            .expect("SOLID primitives");
        assert_eq!(prims.len(), 8, "SOLID changed shape in asset surgery");

        let accessors = json["accessors"].as_array().expect("accessors array");
        let views = json["bufferViews"].as_array().expect("bufferViews array");
        let mut pattern_verts = 0usize;
        for prim in prims {
            let acc = &accessors[prim["attributes"]["POSITION"]
                .as_u64()
                .expect("POSITION accessor") as usize];
            let view = &views[acc["bufferView"].as_u64().expect("bufferView") as usize];
            let base = view["byteOffset"].as_u64().expect("byteOffset") as usize
                + acc.get("byteOffset").and_then(Value::as_u64).unwrap_or(0) as usize;
            let count = acc["count"].as_u64().expect("vertex count") as usize;
            for vertex in 0..count {
                let offset = base + vertex * 12 + 4;
                let world_y = -f32::from_le_bytes(bin[offset..offset + 4].try_into().unwrap());
                assert!(
                    !(0.0015..=0.0135).contains(&world_y),
                    "GROUND_RMUL.glb regained a {:.4}m floor bump; re-run tools/flatten_rmul_floor.py",
                    world_y
                );
                if (0.0005..=0.0015).contains(&world_y) {
                    pattern_verts += 1;
                }
            }
        }
        assert!(
            pattern_verts > 0,
            "the flattened 1mm pattern surfaces disappeared from GROUND_RMUL.glb"
        );
    }
}
