# Fixed-root mesh fallback: execution record

Date: 2026-10-17. Branch: feat/mesh-collider-ccd; starting HEAD d5d07dc.

## Interface and implementation

- Provider bool `rootOverlapFallback=False`; AE label Endpoint Fallback on Root Overlap.
- Python keyword `root_overlap_fallback=False` appended after name, included in reuse.
- Solver read-only `meshRootFallbackActive` describes actual fallback in this evaluation.
- Root proximity is tested before projection, per bone/provider/substep, using
  immutable masked geometry and swept root motion. Scale-dependent hysteresis
  holds until the whole substep clears an extra 5% of root contact radius (or
  the numerical margin). The selected mode is shared by solve and penetrating
  for all iterations. No boneAsCapsule writes or Python per-frame mode switching.
- Live provider identities, policy revisions and solver-local state isolate
  consumers. Source reconnect callbacks invalidate policy history. Non-normal
  DG queries restore playback state on every return path. Full lifecycle rules
  and exact tolerances are documented in mesh_colliders.md.
- A newly connected fallback provider can resolve against its current static
  snapshot immediately. Prior, unobserved surface motion cannot be reconstructed.
- Fallback intentionally leaves the segment middle unprotected. Actual endpoint
  overlaps, infeasible length constraints and CCD failures retain unresolved.

## Tests and runtime

The anchored-root baseline reproduced the failure before implementation. The
new feature/interface cases initially failed against the previous plugin.

Final Maya 2022 results: **51 tests pass** (19 new fallback, 20 mesh, 12 dynamics).
The new suite was also run against the copied plug-ins/2022 artifact. Tests
check actual positions, surface separation, bone length, finite values and
replay, plus default behavior, Python positional compatibility/reuse, persistent
root conflict, middle-only contacts, infeasible endpoints, hysteresis/recovery,
consumer order, multiple/sparse providers, moving/deforming/subframe surfaces,
root trajectory crossings, continuous/timeSamples variants, masks/degenerates,
large world and simulationScale/timeScale, transformed mesh instances,
topology/source changes, context queries, save/load, undo/redo, reset/gap and
connection changes under DG/Serial/Parallel modes. Both AE templates source
successfully in MEL and expose the controls; interactive layout is unverified.

Runtime environment: Maya 2022, API 20220500; Python 3.7.7 (tags/v3.7.7:d7c567b08f, Mar 10 2020, 10:41:24) [MSC v.1900 64 bit (AMD64)].
Actual ABC acceptance plugin: `D:/_Code_Here/Git/MayaTools/boneDynamicsNode/plug-ins/2022/boneDynamicsNode.mll`.
SHA256: `8d1adffcb96f5ef7832e6d128721e6b8a25b72eaadb1f9f7792bba8725c7820f`.

2019 and 2022 Release builds succeed with Visual Studio 18 2026 and the locally
available Maya 2019.3 / 2022.5 devkits. Versioned plug-ins files match their Release
outputs byte-for-byte:

- `plug-ins/2019/boneDynamicsNode.mll`: `26cc30d7850ed46f95a9da045e8fb22f9e1872dc176b7e1d4b4b1aeb8c25c737`
- `plug-ins/2022/boneDynamicsNode.mll`: `8d1adffcb96f5ef7832e6d128721e6b8a25b72eaadb1f9f7792bba8725c7820f`

Maya 2019 retry used H:/Program Files/Autodesk/Maya2019/bin/mayapy.exe
(Python 2.7.11). It prints the before-initialize marker then exits with code 1
inside maya.standalone.initialize, before loading any plugin. Thus 2019 is
compile/synchronization validated, **not runtime validated**. Only 2019/2022
devkits are available locally; other packaged versions were not rebuilt and do
not receive this feature in this task. No interactive viewport/AE or Linux/macOS
runtime validation was performed.

## Real ABC acceptance

Used the original scene_spring_magic.ma, bound_preset_01.mbd.json (Target keys
OFF) and lowbody_anim_cache_01.abc from the sibling MagicBoneDynamic demo_scene.
The 695-vertex collider was imported with AbcImport. Eight chains / 32 segments,
timeScale=0.1, original analytic collider links removed, only the ABC provider
connected. Fresh scene/preset setup for each flag value, sequential frames 0-101.
All 32 segment rows per frame are in `build/root_fallback_abc.json` (local artifact).
The harness is `tests/mesh_root_fallback_abc.py`, with explicit asset/output
arguments; it does not depend on R:/Temp diagnostic scripts or the external test
harness. It asserts finite tips, preserved lengths and reported residual overlaps.

Frame 10, six skirt first segments (gap = distance - tip radius - contactOffset;
world centimeters). All six are unresolved with the flag off, and all six
fallback=True/unresolved=False with it on:

| Chain suffix | Off tip gap | On tip gap | Off length | On length |
|---|---:|---:|---:|---:|
| BL_Armature | 0.319231 | 0.133892 | 0.349351793 | 0.349351791 |
| BR_Armature | 0.143425 | 0.070956 | 0.349351687 | 0.349351710 |
| FL_Armature | 0.287241 | 0.140251 | 0.341955690 | 0.341955711 |
| FR_Armature | 0.293158 | 0.046334 | 0.341844444 | 0.341844460 |
| L_Armature | 0.182832 | 0.150463 | 0.339040453 | 0.339040436 |
| R_Armature | 0.290705 | 0.100195 | 0.339072782 | 0.339072760 |

Across frames 1-101 (counts are segment-evaluations, not unique nodes):

| Setting | All unresolved | Skirt-root unresolved | Fallback | Minimum skirt-root tip gap |
|---|---:|---:|---:|---:|
| False | 624 | 606 | 0 | -0.031361 |
| True | 202 | 94 | 695 | -0.001144 |

Residual collisions are **not all solved**. Enabled all-segment minimum tip gap
is -0.047926; every observed gap below -0.0001 has unresolved=True in both runs.
Maximum same-frame bone-length difference between runs is 1.28e-7. The six skirt
roots remain in fallback; their maximum per-frame tip displacement is 0.558595
(off: 0.887624), so the real animation is not claimed jump-free. Other segments
record 48 exits; these are not used as proof of smooth capsule recovery.
The controlled recovery test checks actual displacement (<0.5 on a length-10
bone, including undamped motion), near-threshold hold, restoration and length.
All frame positions, gaps, lengths, flags and recovery displacements are retained
in the JSON for inspection.

Input SHA256 values checked unchanged by the acceptance harness:

- `bound_preset_01.mbd.json`: `7a30854d9b411f2b5abbf24cb3bd9525a1ab51676182646ab31c523aaf9b7dcb`
- `lowbody_anim_cache_01.abc`: `5dde6fc4ac9a88e02395851ea5152cd45edcc5959086eba26de9532056a63b08`
- `scene_spring_magic.ma`: `3e39771cead2a8ae19dc732cc43b5e32638f33001c30920dc00d0f9a23b4e78b`

## Performance and delivery limits

Local 20-frame Parallel CPU smoke comparison, same benchmark workload, five
capsules (root-clear) and shared provider, including Maya/Python overhead:

| Scene | Off ms/frame | On ms/frame | Geometry builds |
|---|---:|---:|---:|
| Static, 20k triangles, 50 bones | 18.760 | 18.863 | 1 / 1 |
| Deforming, 5k triangles, 25 bones | 36.975 | 37.571 | 81 / 81 |

These single runs are noise-sensitive, not a performance guarantee. The query
adds no per-bone mesh copies, new native builds or dependency cycles. The real
ABC run took 13.18s off / 5.18s on; the changed collision mode reduces repeated
infeasible capsule work, so these numbers are not an isolated overhead estimate.
Set BDN_ROOT_OVERLAP_FALLBACK=1 to repeat the enabled benchmark.

Artifacts: plug-ins/2019/boneDynamicsNode.mll and plug-ins/2022/boneDynamicsNode.mll;
settings/API and AE scripts must be installed alongside a matching rebuilt mll.
No vaccineKiller source changes, pluginLoader copy or source asset writes were
made. Downstream MagicBoneDynamic UI/preset field integration is follow-up work.
