# H2 Bringup (`isaaclab_arena_h2`)

Unitree H2 humanoid with Sharpa Wave dexterous hands in Isaac Lab Arena.
Fixed lower body, PinkIK upper-body control, 44-DOF hand control (22 per hand).

> **Branch:** `rwiltz/h2-bringup` (private -- assets not committed, see below)

## Getting the Branch

Clone the public repo first (this is where Git LFS resolves from), then add
the private fork as a second remote:

```bash
git clone git@github.com:isaac-sim/IsaacLab-Arena.git
cd IsaacLab-Arena
git remote add jiwenc-nv git@github.com:jiwenc-nv/IsaacLab-Arena.git
git fetch jiwenc-nv
git checkout devel/h2-gr00t-ra
```

## Quick Start

### 1. Start the Docker container

```bash
./docker/run_docker.sh
```

### 2. Place assets (see [Asset Setup](#asset-setup) below)

### 3. Start the CloudXR runtime

In a **separate terminal**, start the CloudXR streaming server inside the container.

```bash
./docker/run_docker.sh
python -m isaacteleop.cloudxr
```

Leave this running while you use the teleop environment.

### 4. Run the H2 teleop environment

All commands below are run **inside the container** (after `./docker/run_docker.sh`).

Make sure to run the **source** command printed in the CloudXR terminal.

```bash
# OpenXR teleop with Kit viewer, object and destination
python isaaclab_arena/scripts/imitation_learning/teleop.py \
  --viz kit \
  --device cpu \
  h2_teleop \
  --teleop_device openxr \
  --object brown_box \
  --destination blue_sorting_bin
```

```bash
# Minimal: just teleop, no objects
python isaaclab_arena/scripts/imitation_learning/teleop.py \
  --viz kit \
  --device cpu \
  h2_teleop \
  --teleop_device openxr
```

```bash
# Headless zero-action smoke test (no headset needed)
python isaaclab_arena/evaluation/policy_runner.py \
  --policy_type zero_action \
  --num_steps 100 \
  h2_teleop \
  --embodiment h2_pink
```

## Asset Setup

Assets are **not committed** to this branch. You need to place them manually.
The total size is ~159 MB.

### Untar the assets tarball

If you have the `h2_assets.tar.gz` tarball, extract it into the `isaaclab_arena_h2` directory:

```bash
tar -xzf h2_assets.tar.gz -C isaaclab_arena_h2/
```

This will populate the `isaaclab_arena_h2/assets/` tree described below.

### Required: USD for Isaac Sim (pick one)

The embodiment resolves the USD in priority order. You only need one:

| Priority | Path (relative to `isaaclab_arena_h2/`) | Description |
|----------|------------------------------------------|-------------|
| 1 | `assets/H2_sharpa_flattened/H2_sharpa_flattened_clean.usd` | Flattened USD with Sharpa hands (recommended) |
| 2 | `assets/H2_with_sharpa_hands/H2_with_sharpa_hands.usda` | Converter output (generated from URDF) |
| 3 | `assets/H2_with_hands/H2_with_hands.usda` | Dex3-1 hands variant |
| 4 | `assets/H2/H2.usda` | Body only, no hands |

You can also override by setting `H2_USD_PATH=/path/to/your.usd`.

### Required: URDF + meshes (for PinkIK controller)

The PinkIK controller loads a URDF at runtime for Pinocchio. Resolution order:

| Priority | Path | Description |
|----------|------|-------------|
| 1 | `assets/urdf/H2_with_sharpa_hands.urdf` | Merged URDF (body + Sharpa hands) |
| 2 | `~/repo/robot_menagerie/unitree/h2/urdf/H2_with_sharpa_hands.urdf` | robot_menagerie clone |
| 3 | `assets/urdf/H2_with_hands.urdf` | Dex3-1 hands fallback |
| 4 | `~/repo/robot_menagerie/unitree/h2/urdf/H2.urdf` | Body only fallback |

Override with `H2_URDF_PATH=/path/to/your.urdf`.

**The URDF's mesh directory must be alongside it.** The controller sets
`mesh_path = dirname(urdf_path)`, so mesh STLs must be in `meshes/` relative
to the URDF (e.g. `assets/urdf/meshes/*.stl`, `assets/urdf/meshes/sharpa_left/*.STL`).

### Required for teleop: standalone Sharpa hand URDFs

DexPilot retargeting loads standalone hand URDFs:

- `assets/urdf/sharpa_standalone/left_sharpa_wave.urdf`
- `assets/urdf/sharpa_standalone/right_sharpa_wave.urdf`

### Complete asset tree

```
isaaclab_arena_h2/assets/
├── H2_sharpa_flattened/              # Recommended USD
│   ├── H2_sharpa_flattened_clean.usd
│   └── H2_sharpa_flattened.usd
├── H2_with_sharpa_hands/             # Generated USD (alternative)
│   ├── H2_with_sharpa_hands.usda
│   └── payloads/
├── H2_with_hands/                    # Dex3-1 USD (alternative)
│   ├── H2_with_hands.usda
│   └── payloads/
├── H2/                               # Body-only USD (alternative)
│   ├── H2.usda
│   └── payloads/
├── urdf/
│   ├── H2_with_sharpa_hands.urdf     # Merged URDF (required for PinkIK)
│   ├── H2.urdf                       # Body-only URDF
│   ├── H2_with_hands.urdf            # Dex3-1 URDF
│   ├── H2_simple_colliders.urdf
│   ├── meshes/                       # Body STL meshes
│   │   ├── pelvis.stl
│   │   ├── torso_link.stl
│   │   ├── ...                       # ~32 body STLs
│   │   ├── sharpa_left/              # Left hand STLs (~36 files)
│   │   └── sharpa_right/             # Right hand STLs (~36 files)
│   └── sharpa_standalone/            # For DexPilot retargeting
│       ├── left_sharpa_wave.urdf
│       └── right_sharpa_wave.urdf
├── config.yaml
└── README.md
```

## Generating Assets from Source

If you have the source URDFs instead of the pre-built assets:

### Merge Sharpa hands onto H2 body URDF

Requires `~/repo/sharpa-urdf-usd-xml/wave_01/` (the Sharpa Wave URDF repo):

```bash
python isaaclab_arena_h2/scripts/merge_h2_sharpa_urdf.py
```

### Convert URDF to USD (inside Docker)

```bash
/isaac-sim/python.sh isaaclab_arena_h2/scripts/convert_h2_urdf_to_usd.py
```

## Architecture Overview

```
isaaclab_arena/embodiments/h2/h2.py      # H2PinkEmbodiment (registered as "h2_pink")
                                          # Scene config, PD gains, PinkIK action config,
                                          # observations, events, XR config
                                          #
isaaclab_arena_environments/              # H2TeleopEnvironment (registered as "h2_teleop")
  h2_teleop_environment.py               # galileo_locomanip scene + optional objects
                                          #
isaaclab_arena_h2/                        # H2-specific package
  h2_env/
    robot_model_utils.py                  # URDF path resolution for PinkIK
  teleop/
    h2_pink_pipeline.py                   # 58D IsaacTeleop pipeline (wrist SE3 + DexPilot)
    data/configs/
      sharpa_wave_{left,right}_dexpilot.yml
  scripts/
    merge_h2_sharpa_urdf.py               # Merge body + Sharpa hand URDFs
    convert_h2_urdf_to_usd.py             # URDF -> USD conversion
  assets/                                 # Robot assets (not committed)
```

## Known Limitations

- **Fixed lower body** -- no locomotion / whole-body control policy yet.
- **PD gains are rough estimates** -- derived from URDF effort limits and G1 values;
  need tuning on real hardware.
- **XR anchor offsets** -- copied from G1, need calibration for H2 torso height.
- **Camera offset** -- approximate, needs measurement from H2 CAD.
