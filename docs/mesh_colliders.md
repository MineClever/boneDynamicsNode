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

Validation on this branch: 20 mesh-collider tests and 12 existing regressions pass
in Maya 2022. Release builds succeed for Maya 2019 and 2022. The locally installed
Maya 2019 exits during `maya.standalone.initialize()` before any plugin is loaded;
its runtime could not be validated in this environment. Interactive viewport/AE
behavior still needs checking in an interactive Maya session.
