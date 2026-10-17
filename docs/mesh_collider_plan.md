# Mesh collider implementation

Branch: `feat/mesh-collider-ccd`, based on `feat/simulation-time`.

## Required behavior

- A reusable `meshColliderNode` accepts evaluated Maya world-space mesh data.
- Immutable geometry and MMeshIntersector caches are shared by downstream bones.
- Subframe DG contexts sample upstream deformers without moving the scene clock.
- Triangle CCD covers moving spheres and tapered bone capsules against linearly
  moving triangles between explicit mesh time samples.
- Static meshes reuse acceleration data; deforming meshes invalidate geometry.
- A configurable color set (default `bdnCollisionExclude`) excludes a polygon
  when every face corner has red >= the configured threshold (default 0.5).
  Missing/unassigned colors retain collision. Exclusion precedes triangulation.
- Existing raw `meshCollider[]` connections continue to work. Python mesh
  connections use the new shared collider node by default.
- `simulationScale`, timeScale, reset/seek, DG/serial/parallel evaluation, file
  save/load and topology/mask edits must remain coherent.

## Milestones

1. [done] Native mesh cache, temporal snapshots and plugin registration.
2. [done] Swept triangle broad phase, sphere/capsule CCD and constraint response.
3. [done] Solver integration, Python helpers, AE controls and usage documentation.
4. [done] Maya 2022: 20 mesh-collider tests and 12 existing regression tests pass.
   Maya 2019 and 2022 Release builds pass. Native-cache and deforming-mesh smoke
   benchmarks completed. The installed Maya 2019 exits during standalone
   initialization even without loading this plugin, so 2019 runtime validation
   remains environment-limited (compilation only).

## Acceptance checks

- Fast endpoint crosses a zero-thickness plane without tunneling.
- A plane crosses a stationary endpoint, including an out-and-back subframe motion.
- Skin deformation updates the collision surface; shared consumers see one cache.
- Bone midsection hits a triangle when the endpoint misses it.
- Color masks remove the selected faces, survive file save/load, and update live.
- Static inputs reuse MMeshIntersector; geometry/topology/mask edits invalidate it.
- Sparse/multiple colliders, scale conversion and rollback do not retain stale state.
- Existing regression suite passes, and Maya 2019/2022 build successfully.

## Explicit numerical model

Mesh trajectories are piecewise linear between evaluated time samples; finite
samples cannot reconstruct arbitrary unobserved animation. Topology changes have
no vertex correspondence for CCD: reset contact history and expose a diagnostic.
Anchored roots or mutually conflicting constraints can be infeasible; expose
unresolved-contact status instead of claiming unconditional nonpenetration.
