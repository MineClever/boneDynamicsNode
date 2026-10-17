# Maya Ribbon GPU Collision Acceleration — Agent Engineering Brief

## 1. Project Context

This project is an Autodesk Maya C++ plug-in for bone-driven ribbon simulation.

The current ribbon implementation is primarily CPU-based:

- Simulation is evaluated frame by frame on the CPU.
- Ribbon joints / simulation points are updated sequentially.
- Collision uses the current frame's deformed collision mesh.
- Mesh collision is expected to become the dominant performance cost as:
  - collision mesh triangle count increases;
  - the number of ribbon chains increases;
  - the number of simulation points increases;
  - substeps / solver iterations increase.

The objective is **not** to immediately rewrite the entire ribbon solver as a GPU-only system.

The preferred strategy is to preserve the existing CPU implementation as the reference / fallback path, then progressively move the expensive collision-related stages to Maya's OpenCL/GPU infrastructure.

---

# 2. Primary Goal

Accelerate per-frame ribbon-vs-DeformedMesh collision calculations using the GPU while keeping the existing CPU ribbon solver architecture usable.

The highest-priority optimization targets are:

1. Deformed collision mesh spatial acceleration structure generation.
2. Broad-phase collision queries.
3. Triangle closest-point / penetration calculations.
4. Batched collision processing for multiple ribbon points / chains.
5. Optional later migration of PBD / XPBD constraint solving to GPU.

The initial implementation should **not** require the full ribbon simulation state to remain on the GPU.

---

# 3. Current Baseline

The existing implementation can be treated approximately as:

```text
Maya Timeline
    ↓
Evaluate current frame
    ↓
Get current DeformedMesh
    ↓
CPU ribbon simulation
    ↓
CPU mesh collision tests
    ↓
Update ribbon joint transforms
```

Conceptually:

```cpp
for each frame:
    evaluateDeformedMesh();

    for each ribbon:
        integrateRibbon();

        for each solverIteration:
            solveRibbonConstraints();
            solveMeshCollisions();

        writeJointTransforms();
```

The existing CPU implementation must remain available during GPU development for:

- correctness comparison;
- debugging;
- unsupported GPU environments;
- regression testing;
- deterministic fallback.

---

# 4. Target Architecture

The preferred long-term architecture is:

```text
                         Maya Evaluation Graph

Main Skeleton
     │
     ▼
 skinCluster / deformers
     │
     ▼
 Deformed Collision Mesh
     │
     │ GPU vertex positions
     ▼
┌──────────────────────────────────────────────┐
│ GPU Collision Backend                       │
│                                              │
│ OpenCL                                       │
│                                              │
│  Deformed Vertex Buffer                     │
│          ↓                                   │
│  Triangle AABB Generation                   │
│          ↓                                   │
│  Spatial Acceleration Structure             │
│          ↓                                   │
│  Broad Phase                                │
│          ↓                                   │
│  Point / Segment / Sphere vs Triangle       │
│          ↓                                   │
│  Collision Results                          │
└──────────────────────────────────────────────┘
                  │
                  ▼
        Small GPU → CPU Readback
                  │
                  ▼
         Existing CPU Ribbon Solver
                  │
                  ▼
          Ribbon Joint Matrices
```

The important architectural decision is:

> Keep the expensive mesh collision workload on the GPU, but do not force the entire ribbon solver onto the GPU in the first implementation.

---

# 5. Maya GPU / OpenCL Integration

Prefer Maya's own OpenCL context instead of creating an unrelated OpenCL runtime context.

Relevant Maya APIs to investigate/use include:

```cpp
MOpenCLInfo
MOpenCLBuffer
MAutoCLMem
MAutoCLEvent
MPxGPUDeformer
MGPUDeformerData
MGPUDeformerBuffer
```

The implementation should attempt to reuse Maya's:

```cpp
cl_context
cl_device_id
cl_command_queue
```

through `MOpenCLInfo`.

Conceptually:

```cpp
cl_context context =
    MOpenCLInfo::getOpenCLContext();

cl_device_id device =
    MOpenCLInfo::getOpenCLDeviceId();

cl_command_queue queue =
    MOpenCLInfo::getMayaDefaultOpenCLCommandQueue();
```

Avoid creating a second independent OpenCL context unless there is a strong technical reason.

---

# 6. Critical Performance Requirement

Do **not** design the final system around this per-frame path:

```text
Maya GPU Deformed Mesh
        ↓
GPU → CPU
        ↓
MPointArray
        ↓
CPU → GPU
        ↓
OpenCL Collision
```

This introduces unnecessary synchronization and PCIe transfer overhead.

The desired path is:

```text
Maya GPU Deformed Mesh
        ↓
GPU buffer
        ↓
OpenCL collision kernels
```

The collision mesh vertex positions should ideally remain resident on the GPU.

---

# 7. Development Strategy

The implementation should be divided into multiple phases.

## Phase 0 — Preserve Existing CPU Solver

Do not remove or heavily refactor the existing CPU path before GPU validation.

Required mode:

```text
Collision Backend:
    CPU
    GPU
```

or equivalent internal switching.

The CPU implementation is the ground-truth reference.

---

## Phase 1 — GPU Collision Prototype

The first GPU version may still upload collision mesh positions from CPU every frame.

Pipeline:

```text
DeformedMesh
    ↓
CPU vertex array
    ↓
OpenCL upload
    ↓
GPU broad phase
    ↓
GPU triangle collision
    ↓
collision result readback
    ↓
existing CPU ribbon solver
```

This phase exists to validate:

- OpenCL initialization;
- kernel compilation;
- collision correctness;
- buffer management;
- GPU/CPU result parity;
- Maya timeline behavior.

Do not prematurely optimize GPU interoperability before this stage works reliably.

---

## Phase 2 — Spatial Acceleration

Do not perform brute-force:

```text
RibbonPointCount × TriangleCount
```

collision testing.

The collision system must contain a broad-phase acceleration structure.

Preferred first implementation:

```text
Uniform Grid / Spatial Hash
```

Why:

- collision mesh topology is generally stable;
- only deformed vertex positions change each frame;
- GPU-friendly parallel construction;
- relatively simple implementation;
- well suited to character-sized collision meshes;
- efficient for dozens to thousands of ribbon simulation points.

Pipeline:

```text
Deformed Vertices
      ↓
Triangle AABB
      ↓
Grid Cell Assignment
      ↓
Spatial Hash / Uniform Grid
      ↓
Ribbon Point Cell Query
      ↓
Candidate Triangles
      ↓
Narrow Phase
```

---

# 8. Optional Later Broad Phase

If Uniform Grid performance becomes insufficient because of:

- highly non-uniform geometry distribution;
- very large meshes;
- large scene-space variance;

consider a GPU BVH / LBVH implementation.

Possible pipeline:

```text
Triangle AABB
    ↓
Centroid
    ↓
Morton Code
    ↓
GPU Sort
    ↓
LBVH
    ↓
Traversal
```

Do **not** implement LBVH first unless profiling proves it is necessary.

---

# 9. Collision Mesh Representation

The GPU collision mesh should be represented separately from Maya object abstractions.

Suggested structure:

```cpp
struct CollisionMeshGPU
{
    cl_mem positions;
    cl_mem triangleIndices;

    uint32_t vertexCount;
    uint32_t triangleCount;
};
```

Important distinction:

### Static / infrequently changing data

```text
triangleIndices
topology
rest metadata
```

Upload only when topology changes.

### Per-frame dynamic data

```text
deformed vertex positions
```

Update every evaluation frame.

The implementation should detect topology changes and avoid rebuilding immutable buffers unnecessarily.

---

# 10. Ribbon Simulation Representation

The GPU backend should not depend directly on Maya joint classes.

Do not make OpenCL kernels operate on:

```text
MFnIkJoint
MTransformationMatrix
MMatrix
```

Instead define compact simulation data.

Example:

```cpp
struct RibbonParticle
{
    float position[4];
    float previousPosition[4];
};
```

or an OpenCL-compatible SoA representation.

A ribbon chain conceptually becomes:

```text
Joint0 --- Joint1 --- Joint2 --- Joint3
   ↓         ↓          ↓          ↓
  P0        P1         P2         P3
```

The existing CPU code may continue converting the simulated particle positions into Maya joint transforms.

---

# 11. Recommended CPU/GPU Boundary

For the current project state, the preferred boundary is:

```text
GPU:
    DeformedMesh collision acceleration
    broad phase
    triangle collision math
    optional collision projection

CPU:
    existing ribbon state management
    joint orientation generation
    Maya DG integration
    final joint transform output
```

This keeps migration risk low.

Only a very small result needs to be read back from GPU.

Example:

```text
100 ribbon points × float4
≈ 1.6 KB
```

Even with additional normals, penetration depths, or velocities, the returned data remains small.

Avoid returning the entire deformed mesh from GPU to CPU.

---

# 12. Narrow-Phase Collision

The narrow phase should support at minimum:

```text
Point / Sphere vs Triangle
```

Prefer treating every ribbon simulation point as having a collision radius.

Suggested data:

```cpp
struct RibbonCollisionParticle
{
    float3 position;
    float radius;
};
```

For each candidate triangle:

1. Compute closest point on triangle.
2. Compute distance to particle center.
3. If distance < radius:
   - compute contact normal;
   - calculate penetration depth;
   - accumulate or select collision correction.

Conceptually:

```text
closest = ClosestPointOnTriangle(p, a, b, c)

delta = p - closest
distance = length(delta)

if distance < radius:
    normal = normalize(delta)
    penetration = radius - distance
    correction = normal * penetration
```

Degenerate triangles must be handled safely.

---

# 13. Multiple Contacts

Do not assume a ribbon point has only one triangle contact.

Possible strategies:

### Initial implementation

Use the deepest penetration.

```text
bestContact = max(penetration)
```

### Later implementation

Accumulate / iteratively solve multiple contacts.

For XPBD-style simulation:

```text
constraints
→ collision constraints
→ constraints
→ collision constraints
```

may produce better stability.

---

# 14. OpenCL Kernel Decomposition

Avoid one monolithic OpenCL kernel.

Prefer separate kernels.

Suggested initial kernel set:

```text
Kernel 1
UpdateTriangleAABB

Kernel 2
BuildGridEntries

Kernel 3
Sort / OrganizeGridEntries

Kernel 4
BuildCellRanges

Kernel 5
QueryRibbonParticles

Kernel 6
SolveTriangleCollision
```

If simulation later moves to GPU:

```text
Kernel 7
IntegrateRibbon

Kernel 8
SolveDistanceConstraints

Kernel 9
SolveBendConstraints

Kernel 10
FinalizeVelocity
```

---

# 15. OpenCL Event Synchronization

Do not use excessive:

```cpp
clFinish(queue);
```

between every stage.

That would serialize the GPU pipeline and significantly reduce performance.

Use OpenCL events.

Conceptually:

```text
Deformer Output
      │
      │ event A
      ▼
Triangle AABB
      │
      │ event B
      ▼
Grid Build
      │
      │ event C
      ▼
Collision Query
      │
      │ event D
      ▼
Readback
```

The implementation should:

- wait on upstream Maya GPU buffer events;
- enqueue kernels with dependency events;
- publish completion events correctly;
- synchronize CPU only when result readback is actually required.

---

# 16. Maya GPU Deformer Integration

If the DeformedMesh already lives in Maya's GPU deformation chain, investigate accessing it through:

```text
MPxGPUDeformer
MGPUDeformerData
MGPUDeformerBuffer
```

The long-term preferred data path is:

```text
skinCluster
    ↓
MGPUDeformerBuffer
    ↓
OpenCL collision kernels
```

instead of:

```text
skinCluster
    ↓
MFnMesh::getPoints()
    ↓
OpenCL upload
```

Important:

Reading mesh data through regular CPU geometry access during GPU evaluation may force Maya to fall back to CPU evaluation or introduce synchronization.

The Agent must profile and verify that GPU buffer access does not accidentally trigger CPU evaluation.

---

# 17. Maya Dependency Graph Cycle Risk

The solver must avoid a dependency cycle.

Dangerous example:

```text
Skeleton
    ↓
skinCluster
    ↓
Collision Mesh
    ↓
Ribbon Solver
    ↓
Ribbon Joints
    ↓
same skinCluster
```

This forms:

```text
DG cycle
```

Preferred dependency:

```text
Main Character Skeleton
        ↓
Body Collision Mesh
        ↓
Ribbon Solver
        ↓
Ribbon Skeleton
        ↓
Ribbon Mesh
```

The collision body mesh must not depend on the simulated ribbon joints.

---

# 18. Timeline / Simulation State

The current solver is frame-based.

Maya evaluation order must not be assumed to always be:

```text
1 → 2 → 3 → 4 → 5
```

Users may evaluate:

```text
1 → 80 → 25 → 100
```

or Maya may reevaluate nodes because of DG invalidation.

The solver must explicitly handle:

```text
forward sequential evaluation
time jumps
backward scrubbing
scene reload
parameter changes
cache invalidation
```

Suggested CPU-side state:

```cpp
struct RibbonSimulationState
{
    double currentTime;

    std::vector<RibbonParticle> particles;
    std::vector<RibbonParticle> previousParticles;

    bool valid;
};
```

GPU state should mirror only the parts that need acceleration.

---

# 19. Reset Rules

The solver should reset / invalidate when any of the following changes:

```text
currentTime < previousTime
large forward time jump
ribbon topology changes
collision mesh topology changes
simulation parameters change
root transform discontinuity
scene load / reference reload
GPU context recreation
```

Do not silently continue simulation from an unrelated previous state.

---

# 20. Collision Proxy Support

The implementation should support simpler collision proxies in addition to full DeformedMesh collision.

Recommended primitive colliders:

```text
Sphere
Capsule
Plane
Optional low-poly mesh
```

A character may use:

```text
Head      → sphere
Neck      → capsule
Chest     → capsules / low-poly proxy
Arms      → capsules
Legs      → capsules
```

Primitive collision should be preferred where visual quality allows it.

A full render mesh with 100k+ triangles should not automatically be considered the ideal collision representation.

---

# 21. Performance Priorities

Optimization order:

```text
1. Remove brute-force triangle scans.
2. Minimize GPU ↔ CPU transfers.
3. Reuse static topology buffers.
4. Batch all ribbon points.
5. Batch multiple ribbon chains.
6. Avoid per-ribbon OpenCL dispatch when possible.
7. Avoid repeated OpenCL buffer allocation.
8. Avoid unnecessary clFinish().
9. Profile GPU occupancy and memory access.
10. Only then consider moving full XPBD simulation to GPU.
```

---

# 22. Batch Processing Requirement

Do not launch one GPU job for every joint or every ribbon.

Prefer:

```text
all characters / ribbons
        ↓
flatten simulation points
        ↓
one or few GPU dispatches
```

Example metadata:

```cpp
struct RibbonRange
{
    uint32_t firstParticle;
    uint32_t particleCount;
};
```

GPU arrays:

```text
positions[]
previousPositions[]
collisionRadius[]
ribbonRanges[]
```

This is especially important when each ribbon only contains a small number of joints.

---

# 23. Memory Management

Do not allocate and free GPU buffers every frame.

Create persistent GPU resources.

Example ownership:

```cpp
class OpenCLCollisionBackend
{
public:
    bool initialize();
    void shutdown();

    void ensureMeshCapacity(size_t vertexCount,
                            size_t triangleCount);

    void ensureRibbonCapacity(size_t particleCount);

private:
    cl_mem m_positions;
    cl_mem m_triangles;

    cl_mem m_triangleAABB;
    cl_mem m_gridEntries;
    cl_mem m_cellRanges;

    cl_mem m_ribbonPositions;
    cl_mem m_collisionResults;
};
```

Reallocate only when capacity must grow or topology fundamentally changes.

---

# 24. Suggested C++ Module Structure

Recommended separation:

```text
Maya Plugin
│
├── MayaRibbonNode
│   ├── DG attributes
│   ├── timeline handling
│   ├── cache invalidation
│   └── Maya joint output
│
├── RibbonSolverCPU
│   ├── current simulation implementation
│   ├── integration
│   ├── constraints
│   └── CPU collision fallback
│
├── CollisionBackend
│   ├── interface
│   ├── CPUCollisionBackend
│   └── OpenCLCollisionBackend
│
├── OpenCLRuntime
│   ├── Maya OpenCL context
│   ├── kernel compilation
│   ├── event management
│   └── buffer management
│
└── CollisionGeometry
    ├── triangle topology
    ├── mesh versioning
    └── proxy colliders
```

Suggested interface:

```cpp
class ICollisionBackend
{
public:
    virtual ~ICollisionBackend() = default;

    virtual bool updateMesh(
        const CollisionMeshView& mesh) = 0;

    virtual bool collide(
        const RibbonParticleView& particles,
        CollisionResultView& results) = 0;
};
```

CPU and GPU backends should implement the same functional contract.

---

# 25. GPU Backend Fallback

GPU collision must never be mandatory for scene evaluation.

Fallback conditions may include:

```text
OpenCL unavailable
unsupported device
kernel compilation failure
GPU allocation failure
Maya GPU context unavailable
GPU evaluation disabled
driver error
debug mode
```

Behavior:

```text
GPU failed
    ↓
log warning once
    ↓
switch to CPU backend
```

Do not crash Maya because GPU acceleration is unavailable.

---

# 26. Error Handling

Every OpenCL API call must be checked.

Example:

```cpp
cl_int err = CL_SUCCESS;

err = clSetKernelArg(...);

if (err != CL_SUCCESS)
{
    // Log meaningful information.
    // Do not continue with invalid GPU state.
}
```

Log at minimum:

```text
device name
OpenCL version
kernel build log
failed kernel name
OpenCL error code
mesh size
particle count
```

---

# 27. Debug / Validation Mode

Provide a developer validation mode.

Recommended modes:

```text
CPU Only
GPU Only
CPU + GPU Compare
```

`CPU + GPU Compare`:

1. Evaluate CPU collision.
2. Evaluate GPU collision.
3. Compare:
   - contact state;
   - closest point;
   - normal;
   - penetration;
   - corrected position.

Suggested tolerances:

```text
position error:
~1e-4 to 1e-5 Maya units
depending on scene scale
```

Do not require bitwise equality between CPU and GPU floating-point results.

---

# 28. Profiling

Measure stages independently.

Required timing categories:

```text
CPU mesh extraction
CPU → GPU upload
triangle AABB
grid construction
broad phase
narrow phase
GPU → CPU readback
CPU ribbon solve
joint output
total evaluation
```

Without stage-level profiling, do not assume the GPU version is faster.

---

# 29. Expected Performance Characteristics

GPU acceleration becomes increasingly useful when:

```text
collision mesh triangles:
50k – 200k+

ribbon particles:
hundreds+

multiple ribbons:
many simultaneous chains

solver substeps:
2 – 8+

collision iterations:
multiple
```

GPU acceleration may not be beneficial for extremely small cases such as:

```text
one ribbon
10–20 joints
very low-poly collision mesh
```

because dispatch and synchronization overhead may dominate.

---

# 30. Recommended Initial Milestone

The first meaningful production milestone should be:

```text
Existing CPU Ribbon Solver
        +
OpenCL Mesh Collision Backend
        +
Uniform Grid Broad Phase
        +
GPU/CPU Backend Switch
        +
CPU/GPU Validation Mode
```

Do not initially rewrite:

```text
joint matrix generation
timeline state management
ribbon constraint solving
Maya DG integration
```

unless profiling demonstrates those areas are significant bottlenecks.

---

# 31. Recommended Second Milestone

After the OpenCL collision backend is stable:

```text
Maya GPU Deformation Buffer
        ↓
direct OpenCL consumption
```

Goal:

Remove:

```text
MFnMesh::getPoints()
CPU mesh copy
CPU → GPU mesh upload
```

from the common GPU evaluation path.

---

# 32. Recommended Third Milestone

Only after collision is optimized and profiling still shows CPU simulation cost:

Move to:

```text
GPU integration
GPU distance constraints
GPU bend constraints
GPU collision
GPU XPBD iterations
```

Then read back only:

```text
particle positions
or
joint transforms
```

for Maya output.

---

# 33. Possible Future GPU Solver

Possible future pipeline:

```text
Input Ribbon Anchors
        ↓
Integrate
        ↓
Distance Constraints
        ↓
Bend Constraints
        ↓
Collision
        ↓
Distance Constraints
        ↓
Collision
        ↓
Finalize Velocity
        ↓
Small Readback
        ↓
Joint Matrices
```

This should be treated as a later optimization, not the first GPU implementation.

---

# 34. Agent Implementation Rules

The Agent should follow these rules:

1. Do not remove the CPU implementation.
2. Do not rewrite unrelated ribbon solver code during initial GPU work.
3. Establish CPU/GPU numerical comparison before optimization.
4. Avoid brute-force point × triangle collision.
5. Prefer Uniform Grid / Spatial Hash as the first GPU broad phase.
6. Reuse Maya's OpenCL context when possible.
7. Reuse persistent buffers.
8. Keep topology buffers persistent.
9. Update only dynamic vertex positions every frame.
10. Batch multiple ribbon particles in each GPU dispatch.
11. Avoid `clFinish()` except where actual CPU synchronization is required.
12. Use OpenCL events for kernel dependencies.
13. Never introduce a DG dependency cycle.
14. Handle non-sequential Maya timeline evaluation explicitly.
15. Fall back to CPU if GPU execution is unavailable or fails.
16. Profile before and after each major optimization.
17. Keep the code modular so the collision backend can eventually be replaced by CUDA, Vulkan Compute, or another backend if required.

---

# 35. Success Criteria

The GPU collision implementation is considered successful when:

### Correctness

- CPU and GPU collision results agree within tolerance.
- No visible ribbon penetration regression.
- Backward timeline scrubbing works.
- Frame jumps do not corrupt state.
- Scene reload does not preserve invalid GPU state.
- GPU failure safely falls back to CPU.

### Performance

For sufficiently complex meshes / multiple ribbons:

```text
GPU collision time
<
CPU collision time
```

and preferably:

```text
total ribbon evaluation time
significantly reduced
```

The optimization should remain useful after accounting for:

```text
upload
dispatch
synchronization
readback
```

### Architecture

- CPU solver remains intact.
- GPU collision backend is isolated.
- Maya-specific logic is separated from OpenCL kernels.
- No unnecessary per-frame buffer allocation.
- No unnecessary full-mesh GPU → CPU readback.

---

# 36. Final Recommended Direction

For the current project, use the following migration path:

```text
CURRENT
────────────────────────────────────────

CPU DeformedMesh access
        ↓
CPU ribbon simulation
        ↓
CPU collision


STEP 1
────────────────────────────────────────

CPU DeformedMesh access
        ↓
GPU mesh upload
        ↓
OpenCL Spatial Hash
        ↓
OpenCL Triangle Collision
        ↓
small result readback
        ↓
existing CPU ribbon solver


STEP 2
────────────────────────────────────────

Maya GPU DeformedMesh Buffer
        ↓
OpenCL Spatial Hash
        ↓
OpenCL Triangle Collision
        ↓
small result readback
        ↓
existing CPU ribbon solver


OPTIONAL STEP 3
────────────────────────────────────────

Maya GPU DeformedMesh Buffer
        ↓
OpenCL Spatial Hash
        ↓
OpenCL Collision
        ↓
OpenCL XPBD Ribbon Solver
        ↓
small joint/result readback
        ↓
Maya joint output
```

The preferred immediate engineering target is **STEP 1**, followed by profiling and correctness validation.

The preferred long-term production target is **STEP 2**.

STEP 3 should only be implemented if profiling proves that the remaining CPU ribbon solver is still a meaningful bottleneck.
