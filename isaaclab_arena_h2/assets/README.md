# H2 Robot Assets

This directory holds **generated** USD files for the Unitree H2 robot. These are
not checked into git -- each developer generates them locally from the URDF.

## Prerequisites

Clone the `robot_menagerie` repo so the H2 URDF and meshes are available:

```
~/repo/robot_menagerie/unitree/h2/urdf/H2_with_hands.urdf   # H2 body + Dex3-1 hands
~/repo/robot_menagerie/unitree/h2/urdf/meshes/               # STL meshes (body + hand)
```

## Generate the USD (one-time, inside Docker)

```bash
/isaac-sim/python.sh isaaclab_arena_h2/scripts/convert_h2_urdf_to_usd.py
```

This reads from `~/repo/robot_menagerie/unitree/h2/urdf/H2_with_hands.urdf` and
writes the USD into this directory (`isaaclab_arena_h2/assets/`). The H2 embodiment
auto-discovers it at runtime.

To use a different URDF path:

```bash
/isaac-sim/python.sh isaaclab_arena_h2/scripts/convert_h2_urdf_to_usd.py \
    --urdf_path /path/to/your/H2_with_hands.urdf
```

## Convention

This follows the same pattern as the G1 embodiment, which loads assets from
NVIDIA Nucleus at runtime (not from the git repo). Once the H2 assets are
hosted on Nucleus, the conversion script will no longer be needed.
