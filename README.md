# boneDynamicsNode

![Maya](https://img.shields.io/static/v1?message=Maya&color=0696D7&logo=Autodesk&logoColor=white&label=) ![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)

**boneDynamicsNode** is a custom node for Autodesk Maya that enables dynamic simulation on joint chains. It is designed for easy integration into rigs and supports features such as collision handling, angle limits, target pose manipulation, and more.

> [!WARNING]
> This plug-in uses internal-use Maya node IDs:  
> `0x7b001`–`0x7b006`, and `0x7b009` (including custom mesh-collider data).

## ✨ Features

- Simple integration with standard Maya joints
- Multiple collision types: sphere, capsule, infinite plane, ground, and mesh
- Shared deforming mesh colliders with sampled motion, sphere/capsule CCD and vertex-color face exclusion
- Optional [endpoint fallback on fixed-root mesh overlap](docs/mesh_colliders.md#temporary-endpoint-fallback-for-fixed-roots), default off; the bone middle loses collision protection while active
- Dedicated collider nodes and visualizer node
- Python helper package for scripted setup
- Angle limits
- Target pose control
- Transform cancellation for specific nodes
- Per-section scaling
- Turbulent wind
- Support for branching structures

## 🔧 Installation

Download the release zip from the [Releases page](https://github.com/akasaki1211/boneDynamicsNode/releases).

After extracting the zip file:

1. Copy the `boneDynamicsNode.mll` file for your Maya version from the `plug-ins` folder to `C:\Users\<USERNAME>\Documents\maya\<MAYAVERSION>\plug-ins`.
2. Copy the `scripts` folder to `C:\Users\<USERNAME>\Documents\maya`.  
   - The `scripts` folder installs the `bdn` Python helper package and both dynamics/mesh-collider AE templates.

For additional instructions, refer to the [Installation](https://github.com/akasaki1211/boneDynamicsNode/wiki/Installation) wiki page.

## 🖥️ Platform Status

- **Windows:** Developed and tested on Windows. Pre-built plug-ins are provided for Windows.
- **Linux:** Build verified under WSL. Runtime testing in Maya on Linux has not been performed.
- **macOS:** Not tested.

## 🚀 Try it out

Please try running the following script. It creates a single-section joint with bone dynamics applied.  

```python
from maya import cmds

cmds.select(cl=True)
bon = cmds.joint(p=[0,0,0])
end = cmds.joint(p=[10,0,0])

import bdn
bdn.create_dynamics_node(bon, end)

cmds.currentTime(1)
cmds.select(bon)
```

> [!IMPORTANT]
> The simulation advances by scene time (deltaTime). Use Play every frame for a stable preview. `timeScale` changes elapsed simulation time; `simulationScale` changes solver geometry. Both default to 1.

At `timeScale=1`, each scene frame uses one integration step. Damping retains
its dev definition and default: a fraction in [0, 1] removed from velocity per
step (default 0.1). Stiffness is the fraction in [0, 1] removed from the proposed
position displacement per step. Elasticity, mass, forces, angle limits and
turbulence retain their dev formulas; turbulence uses the scene-frame seed.
These parameters are not per-second decay rates. Changing the scene frame rate
resets simulation; the elapsed step duration follows the scene frame rate.
At 30 fps and default scales, scenes without mesh collisions reproduce dev's
30 fps integration. Other scene rates use their actual frame duration rather
than dev's former fixed `fps=30` default.

Mesh CCD subdivides collision detection separately from dynamics integration;
its detection steps do not reapply damping, stiffness or external forces.

Values of `timeScale` above 1 divide the full scaled duration into steps no longer
than one scene frame, without resetting the
simulation or dropping scaled time. Computation cost grows with the multiplier; very high values may not play
in real time. A value of 0 pauses simulation and does not accumulate elapsed time.
The existing seek policy uses **unscaled scene time**: backwards seeks and gaps
over 1/15 second reset the state. Use Play every frame to avoid playback skips.
Scene inputs are sampled once per evaluation; faster motion can still exceed
what the viewport frame rate can show.

`simulationScale` controls the size used by the solver independently of
`timeScale`. Its default is `1.0` and its minimum is `0.000001`. World positions,
bone lengths, radii, collider geometry, ground height, mesh collision cutoff and
offset translations are multiplied by this value before solving. The simulated
position is converted back to world space for the output rotation; scene objects
and collider previews retain their actual size.

For example, `simulationScale=0.01` makes a 100 m section simulate at a 1 m
geometric scale. Gravity, additional force and turbulence strength retain their
numeric values in simulation space, so reducing the scale increases their effect
relative to the bone length. Nonlinear elasticity uses the scaled distances;
mass, damping and angle limits retain their parameter values. This is an artistic
dynamics control, rather than an automatic conversion of Maya's display units.
Changing the value resets simulation on its next evaluation, including when
changed by animation. Set it before replaying from the reset frame.

```python
from maya import cmds
cmds.setAttr("boneDynamicsNode1.simulationScale", 0.01)
# Or pass simulationScale=0.01 to bdn.create_dynamics_node(bon, end, ...).
```

Sphere `radius` and capsule `radiusA`, `radiusB`, and `height` support animated
viewport previews. The draw overrides read their current values on each viewport
update, including during Evaluation Manager playback.

For sampled deforming mesh collision, whole-bone capsules, thin-surface CCD,
and vertex-color face exclusion, see [Mesh colliders](docs/mesh_colliders.md).
`bdn.connect_colliders(node, ['meshTransform'])` creates/reuses a native shared
`meshColliderNode`. Existing direct `meshCollider[]` inputs retain legacy behavior.

Regression checks can be run with Maya's Python interpreter:

```powershell
& 'C:/Program Files/Autodesk/Maya2022/bin/mayapy.exe' tests/maya_regression.py "$PWD/build/win64_2022/Release/boneDynamicsNode.mll"
```

Run `sample_scripts/animated_colliders.py` in interactive Maya to verify the
animated wireframes during playback and scrubbing. Batch checks validate solver
results and animated bounding boxes; they do not render the viewport.

Refer to [Quick Start](https://github.com/akasaki1211/boneDynamicsNode/wiki/Quick-Start) and [Basic Usage](https://github.com/akasaki1211/boneDynamicsNode/wiki/Basic-Usage) for details.

## 📖 Documentation

Full documentation is available in the [wiki](https://github.com/akasaki1211/boneDynamicsNode/wiki/Home):

- [Installation](https://github.com/akasaki1211/boneDynamicsNode/wiki/Installation)
- [Quick Start](https://github.com/akasaki1211/boneDynamicsNode/wiki/Quick-Start)
- [Basic Usage](https://github.com/akasaki1211/boneDynamicsNode/wiki/Basic-Usage)
- [Dynamics Parameters](https://github.com/akasaki1211/boneDynamicsNode/wiki/Dynamics-Parameters)
- [Visualizer](https://github.com/akasaki1211/boneDynamicsNode/wiki/Visualizer)
- [Collision Guide](https://github.com/akasaki1211/boneDynamicsNode/wiki/Collision-Guide)
- [Advanced Features](https://github.com/akasaki1211/boneDynamicsNode/wiki/Advanced-Features)
- [Practical Examples](https://github.com/akasaki1211/boneDynamicsNode/wiki/Practical-Examples)
- [FAQ & Troubleshooting](https://github.com/akasaki1211/boneDynamicsNode/wiki/FAQ-&-Troubleshooting)
- [Change Log](https://github.com/akasaki1211/boneDynamicsNode/wiki/Change-Log)

## 🧰 Third-party Tools

Several third-party tools have been created for working with boneDynamicsNode.
Many thanks to the developers for creating and sharing these tools!

- [nkBdnEditor](https://github.com/imaoki/nkBdnEditor) by [imaoki](https://x.com/imaoki344541)
- [G3DSetupBoneDynamics](https://booth.pm/ja/items/8704160) by [GACCHI](https://x.com/GACCHI_3DCG)

## 📦 Individual Plug-ins

If you only need to open and evaluate scenes that are already set up with boneDynamicsNode, the main `.mll` plug-in file is enough.

Pre-built `boneDynamicsNode.mll` files are available in the [plug-ins](./plug-ins) directory. Download the file that matches your Maya version and place it in your Maya plug-ins path.

**Click "Download raw file" from the linked file page.**

- [Maya 2022 Update 5 Win64](./plug-ins/2022/boneDynamicsNode.mll)
- [Maya 2023 Update 3 Win64](./plug-ins/2023/boneDynamicsNode.mll)
- [Maya 2024 Update 2 Win64](./plug-ins/2024/boneDynamicsNode.mll)
- [Maya 2025 Update 3 Win64](./plug-ins/2025/boneDynamicsNode.mll)
- [Maya 2026 Update 3 Win64](./plug-ins/2026/boneDynamicsNode.mll)
- [Maya 2027 Update 1 Win64](./plug-ins/2027/boneDynamicsNode.mll)

If you want to create new setups using the `bdn` Python helper package or use the Attribute Editor template, follow the instructions in the [Installation](#-installation) section.
