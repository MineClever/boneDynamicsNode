# Sampled mesh colliders

`meshColliderNode` accepts an evaluated world-space Maya mesh and publishes
immutable collision data to any number of `boneDynamicsNode` consumers. The mesh
itself is the viewport preview; the provider is a dependency node, not a locator.
No third-party collision or acceleration library is required.

## Connecting meshes

```python
import bdn

# Simple: mesh transforms and shapes are accepted. A provider is shared when its
# source and settings match an existing provider.
bdn.connect_colliders('boneDynamicsNode1', ['bodyMesh'])

# Explicit provider, useful when many chains must share custom settings:
collider = bdn.create_mesh_collider(
    'bodyMesh',
    exclude_color_set='bdnCollisionExclude',
    time_samples=4,
    contact_offset=0.001,
    continuous=True,
    root_overlap_fallback=False,  # optional fixed-root conflict workaround
)
bdn.connect_colliders('boneDynamicsNode2', [collider])
bdn.connect_colliders('boneDynamicsNode3', [collider])
```

The helper resolves the selected mesh DAG instance, ignores intermediate shapes,
and connects `worldMesh[instance]` plus `time1.outTime`. A transform passed to
`create_mesh_collider` must have one visible mesh shape. The new provider output
connects to `boneDynamicsNode.meshColliderData[]`; sparse and multiple connections
are supported. Existing `meshCollider[]` connections remain a legacy endpoint
collision mode with `meshColCutoff`, and now use cached native point queries.
Reconnect a legacy mesh through the helper to enable the new features.

`boneAsCapsule=False` collides the endpoint sphere. `boneAsCapsule=True` collides
the entire segment swept by linearly varying `rootRadius` and `radius`. Contact
response preserves the bone length. Surface contacts take precedence over angle
limits when both constraints cannot be satisfied simultaneously. `iterations=0`
disables collision processing, as with the original solver.

## Temporary endpoint fallback for fixed roots

`meshColliderNode.rootOverlapFallback` defaults to **False** (including old
scenes). Enable it in the AE with **Endpoint Fallback on Root Overlap**, or use
`bdn.create_mesh_collider(mesh, root_overlap_fallback=True)`. The new Python
argument is appended after `name`; positional calls retain their meaning, and
provider reuse includes this setting.

When `boneAsCapsule=True`, a fixed root sphere overlapping this mesh's valid
surface selects endpoint collision for this bone/provider/substep before any
capsule projection. It does not change `boneAsCapsule`, animate a mode switch,
or affect other providers or analytic colliders. **During fallback the bone's
middle loses collision protection.** The root continues following animation.
The endpoint still uses its radius plus contactOffset, existing CCD and moving
surface response, fixed-root bone length, and multi-mesh final checks. Genuine
endpoint infeasibility, remaining penetration and CCD failures still report
`meshCollisionUnresolved`.

The root query uses the same masked, nondegenerate world-space triangles as the
solver, never a closed-volume inside test. It sweeps the root against all mesh
sample intervals even when endpoint `continuous=False`; mesh sampling itself
still follows `continuous/timeSamples`. Unobserved nonlinear motion cannot be
reconstructed when sampling is disabled. Newly connected providers with only a
current snapshot use that surface immediately; previous surface motion is unknown.

Let R = rootRadius + contactOffset and S = max(R, world bone length). Enter when
the swept root is within R - S*1e-7 (subject to the existing CCD numeric tolerance).
Once active, require the **entire substep** to clear
R + max(16*S*1e-7, 0.05*R) before restoring capsule mode. These distances scale
with scene geometry; no frame count or fixed centimeter hysteresis is used.
The effective mode is selected once per substep and reused by every `solve` and
`penetrating` pass. A single evaluation can contain both fallback and recovered
substeps.

Hysteresis belongs to each solver, keyed by live provider identity and policy
revision. Reset/backward/gap/FPS/simulationScale changes clear it; radius changes,
provider policy/source edits, topology loss and disconnection/reconnection
invalidate affected history. `boneAsCapsule=False` and disabled simulation clear
the relevant state. Non-normal DG context queries use temporary simulation and
fallback state and restore normal playback state on return. Immutable shared
timelines are never mutated by consumers, and policy changes retain geometry caches.

Read-only `boneDynamicsNode.meshRootFallbackActive` reports whether any provider
actually used fallback during **this evaluation**. It is False on reset, disabled
processing, or a same-time refresh with no simulation steps; cached reads retain
the diagnostic associated with the last computed output. It is not a mode input
and does not suppress real failures. See the [validation record](mesh_root_fallback_done.md).

## Motion and deformation

The provider evaluates upstream mesh data at subframe `MDGContext`s without
changing the global scene time. This covers transform animation, independently
driven skinClusters and other deformers that support contextual evaluation.
`timeSamples` (1-64, default 4) is the number of motion intervals over the last
evaluated scene interval. Each interval uses linearly moving triangles. Increasing
it captures more nonlinear deformation, rotation, or out-and-back motion.

Continuous detection uses swept triangle bounds and conservative advancement
with a relative-motion speed bound. Tapered capsule distance is minimized over
the complete segment, not a sparse chain of sample spheres. Moving contacts are
carried with the surface. `MMeshIntersector` supplies thread-safe closest-point
queries; a native swept-AABB tree provides the candidate queries that Maya's
point-query interface does not expose. No external BVH/physics library is used.

Geometry extraction and native cache construction are serialized. Downstream
queries consume immutable snapshots. Unchanged static meshes reuse their native
intersector and triangle tree. Repeated same-time requests retain the published
motion interval, so all bones share the same history. Deformation, topology and
color-mask edits invalidate affected geometry. Published snapshots remain alive
while readers use them. Custom collider data is not serialized: connections and
settings survive save/load, and caches rebuild on demand. Cached Playback is
disabled for this stateful provider.

`simulationScale` applies to integration state; mesh queries and `contactOffset`
use world-space distances. Input/output conversion keeps collision geometry
consistent. `timeScale` changes simulated duration and substep count, while mesh
animation continues to follow actual scene time. Backwards seeks and scene gaps
over 1/15 second reset motion history, matching the existing solver's seek policy.
At normal speed dynamics uses one step per scene frame, retaining dev's damping,
stiffness and force definitions. CCD separately subdivides the predicted motion
into intervals no longer than 1/240 second. These collision intervals do not
apply damping, stiffness, forces or turbulence again; they carry contact
corrections forward along the predicted path.

## Excluding faces with vertex colors

Set `excludeColorSet` to a Maya color-set name. Default: `bdnCollisionExclude`.
Use an RGB/RGBA set; only the red channel is read. A polygon is excluded if **all
of its face-corner red values are >= `excludeThreshold`** (default 0.5). All of
that polygon's triangles are excluded from both nearest-point and CCD queries.

- Black (red=0): collision enabled.
- Red (red=1): collision excluded when every corner of the polygon is painted.
- Missing color set or unassigned colors: collision enabled.
- Empty `excludeColorSet`: ignore vertex colors, collide with all valid faces.

Use face-vertex colors for sharp face selections; shared vertex colors also
affect adjacent faces. For example:

```python
from maya import cmds
cmds.polyColorSet('bodyMesh', create=True, colorSet='bdnCollisionExclude')
cmds.polyColorSet('bodyMesh', currentColorSet=True, colorSet='bdnCollisionExclude')
cmds.polyColorPerVertex('bodyMesh.vtx[*]', rgb=(0, 0, 0))
# Paint the desired region red with Maya's vertex-color tools. For a whole mesh:
# cmds.polyColorPerVertex('bodyMesh.vtx[*]', rgb=(1, 0, 0))
```

## Settings and diagnostics

| Attribute | Meaning |
|---|---|
| `continuous` | Enable CCD; disabled mode tests sampled endpoint surfaces |
| `timeSamples` | Mesh motion intervals per scene evaluation, default 4 |
| `contactOffset` | Extra surface clearance, distance attribute, default 0.001 cm |
| `excludeColorSet` | Name of the face-exclusion color set |
| `excludeThreshold` | Inclusive red-channel cutoff, default 0.5 |
| `geometryBuilds` | Cumulative native intersector builds for this provider |
| `sampleCount` | Number of geometry samples in the published interval |
| `topologyChanged` | Vertex/triangle correspondence was lost in an interval |
| bone's `meshCollisionUnresolved` | Unresolved overlap, infeasible root/contact, CCD iteration limit, or lost correspondence |

The new surface collision path is two-sided and works with open/thin meshes.
Start from a nonpenetrating pose for dependable CCD. It is not a volume-inside
classifier for recovering a bone initialized deep inside a closed mesh.

Finite samples cannot recover animation between unseen times. Stateful third-party
deformers may not support subframe contexts; a failed sample returns an evaluation
error rather than silently claiming valid CCD. Topology/mask changes can destroy
triangle correspondence: use the current surface for that interval and report
`topologyChanged`. A fixed bone root intersecting a surface, competing colliders,
or incompatible angle/length/contact constraints can be infeasible; inspect
`meshCollisionUnresolved` instead of assuming unconditional nonpenetration.

Use an independently driven collision mesh. A mesh skinned to the same dynamic
bones and fed back into their collider creates a Maya dependency cycle; this is
not a self-collision solver.

## Verification

```powershell
& 'C:/Program Files/Autodesk/Maya2022/bin/mayapy.exe' tests/mesh_collider_regression.py "$PWD/build/win64_2022/Release/boneDynamicsNode.mll"
& 'C:/Program Files/Autodesk/Maya2022/bin/mayapy.exe' tests/maya_regression.py "$PWD/build/win64_2022/Release/boneDynamicsNode.mll"
& 'C:/Program Files/Autodesk/Maya2022/bin/mayapy.exe' tests/mesh_root_fallback_regression.py "$PWD/build/win64_2022/Release/boneDynamicsNode.mll"
& 'C:/Program Files/Autodesk/Maya2022/bin/mayapy.exe' tests/mesh_collider_benchmark.py "$PWD/build/win64_2022/Release/boneDynamicsNode.mll"
```

The benchmark includes Maya/Python evaluation overhead, excludes viewport drawing,
and updates scene time once per frame. Initial local measurements (20 frames,
Parallel evaluation, five capsules in each scene):

| Mesh | Bones | Time/frame | Native builds |
|---|---:|---:|---:|
| Static, 20,000 triangles | 50 | ~19.7 ms | 1 total |
| Deforming, 5,000 triangles | 25 | ~51.9 ms | 81 total (initial + 4/frame) |

These are smoke measurements on the development machine, not performance promises.
Deforming surfaces with many bones or high sampling rates need scene-specific profiling.

Validation on this branch: 19 fallback tests, 20 mesh-collider tests and 12 existing
regressions pass in Maya 2022. Release builds succeed for Maya 2019 and 2022. The locally installed
Maya 2019 exits during `maya.standalone.initialize()` before any plugin is loaded;
its runtime could not be validated in this environment. Interactive viewport/AE
behavior still needs checking in an interactive Maya session.
