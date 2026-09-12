# AGENTS.md

## Project

**Daedalus** (`daedalus`) — a RoboMaster vision-algorithm verification simulator built with Bevy 0.19 + avian3d physics (Rust, edition 2024, AGPL-3.0). It simulates the battlefield (power rune, outpost, armor plates, vehicles) and feeds a vision/auto-aim stack through two alternative channels: ROS2 topics (r2r) or Talos shared-memory IPC. The README is in Chinese and documents published/subscribed topics and keybindings — read it before touching the ROS2/Talos interface code.

## Layout

- `src/` — main Bevy app
  - `robomaster/` — domain simulation (armor, outpost, power_rune, tech_core, vehicle)
  - `capture/` — off-screen color/depth capture pipeline (driver, view_copy, depth)
  - `ros2/` — ROS2 integration; compiled only with `--features ros2`
  - `talos/` — Talos IPC integration; compiled with default features
  - `systems/` — gameplay systems (input, controller, camera, projectile, uav)
  - `config.rs` — `config.toml` loading; the file is **hot-reloaded at runtime** via a notify watcher (only physics params are re-applied; `[ros2]` keys are read once at startup)
  - `metalfx.rs` — macOS-only MetalFX upscaling plugin
- `crates/exact` — tiny helper for exact-size array conversions (no deps)
- `crates/talos-ipc` — shared-memory IPC (memmap2, triple buffer, pub/sub)
- `rm_interfaces/` — ROS2 ament package with `.msg`/`.srv` definitions
- `third_party/r2r_msg_gen` — vendored patch of r2r_msg_gen (r2r 0.9.6 is pinned to a git tag in Cargo.toml via `[patch.crates-io]` because it was never published to crates.io; the vendored crate is wired via `[patch."https://github.com/sequenceplanner/r2r"]` to override the git checkout's copy — moving it back to `[patch.crates-io]` breaks the ros2 build)
- `config.toml` — runtime config (window present mode, capture resolutions, physics Hz, vehicle/projectile params)
- `docs/` — developer docs (`build.md` build guide incl. the Windows/pixi workflow, `ros2-autoaim.md` ROS2 auto-aim dev guide)

## Build / Test

```sh
cargo run                                # default features = talos only
cargo run --no-default-features --features ros2   # ROS2 integration; needs a sourced ROS2 env
cargo test                               # inline unit tests (capture driver, physics components)
cargo fmt                                # required: pre-commit hook runs `cargo fmt --check`
cargo clippy                             # clippy.toml raises type-complexity-threshold to 500
```

- Features: `ros2`, `talos` (default), `ffmpeg`. The extra binary `talos_gimbal_mock_server` requires `--features talos,ffmpeg`.
- `cargo-wrapper.sh` is a zsh dev convenience (sources gitignored `env.sh`, runs gitignored `build.sh` when ROS2 output is requested, defaults to `--release`). `build.sh`/`env.sh` are user-local and not in the repo — don't expect them to exist.

## Conventions & Gotchas

- The `ros2` and `talos` modules are `#[cfg(feature = ...)]`-gated in `src/main.rs`; keep new integration code inside the matching module so the other feature still compiles.
- When ROS2 capture is active, the Talos plugin is skipped automatically; `DAEDALUS_FORCE_TALOS_CAPTURE=1` overrides this (see `should_enable_talos_plugin` in `src/main.rs`).
- Dev profiles: `opt-level = 1` for the app, `3` for dependencies; Bevy uses `dynamic_linking` for faster iteration.
- Platform-specific code exists for macOS (MetalFX via objc2-metal in `src/metalfx.rs` and Cargo.toml) and WSL (wgpu non-compliant adapter workaround in `src/main.rs`). The ros2 feature needs a sourced ROS2 environment; it builds on Linux and on Windows (see `docs/build.md` for the verified Windows/pixi-lyrical workflow).
- `dupast.toml` configures a duplicate-code detector over `src/` — avoid copy-pasting between the per-entity `robomaster/*` modules; they share structure intentionally but drift causes noise.
- Rendering defaults to `auto_no_vsync` so off-screen capture can exceed 60Hz; debug overlays (egui/inspector/diagnostics) are off by default because they reduce FPS.
