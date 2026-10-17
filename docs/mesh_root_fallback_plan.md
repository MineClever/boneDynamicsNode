# Fixed-root mesh overlap fallback

Base: d5d07dc on feat/mesh-collider-ccd. Worktree was clean. Requirements read
from the user's R:/RamCache/boneDynamicsNode_mesh_root_overlap_fallback_agent_prompt.md.

1. [done] Reproduce fixed-root capsule infeasibility before implementation.
   Baseline test passes; new interface tests fail against the previous binary.
2. [done] Add default-off provider policy, immutable publication, solver-local
   provider-specific hysteresis, pre-projection swept root queries and one
   effective mode shared by solve/final checks; expose evaluation diagnostics.
3. [done] Append Python argument, distinguish reuse settings, add AE controls,
   document lifecycle and loss of middle protection.
4. [done] Exercise synthetic geometry, motion/deformation, masks, instances,
   evaluation contexts, replay, connections, settings, and real ABC input.
5. [done] Build available 2019/2022 targets, synchronize versioned artifacts,
   retry 2019 initialization and record runtime limits separately.

No third-party library or MagicBoneDynamic implementation change. Original scene,
ABC and preset remain unchanged. UI/preset integration in that tool is follow-up
work. Acceptance results and remaining limits are in mesh_root_fallback_done.md.
