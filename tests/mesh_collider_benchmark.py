"""mayapy tests/mesh_collider_benchmark.py <rebuilt .mll>.

Reproducible CPU smoke benchmark, including Maya/Python evaluation overhead.
This measures representative workloads; it does not enforce machine-specific FPS.
"""
import json
import os
import runpy
import time

suite = runpy.run_path(os.path.join(os.path.dirname(__file__), 'mesh_collider_regression.py'))
cmds, bdn = suite['cmds'], suite['bdn']
results = []
fallback = os.environ.get('BDN_ROOT_OVERLAP_FALLBACK', '0') == '1'
for deforming, subdivisions, count in ((False, 100, 50), (True, 50, 25)):
    cmds.file(new=True, force=True)
    cmds.currentUnit(time='ntsc', linear='cm')
    cmds.evaluationManager(mode='parallel')
    mesh = suite['plane'](width=100, subdivisions=subdivisions)
    if deforming:
        cluster, handle = cmds.cluster(mesh + '.vtx[*]')
        cmds.setKeyframe(handle, attribute='translateY', time=0, value=0)
        cmds.setKeyframe(handle, attribute='translateY', time=20, value=0.25)
    collider = bdn.create_mesh_collider(mesh, root_overlap_fallback=fallback)
    bones = []
    for i in range(count):
        node, end = suite['bone'](root=(-10, 2, -20 + i * 0.8), capsule=i < 5)
        cmds.setAttr(node + '.additionalForceY', -1000)
        bdn.connect_colliders(node, [collider])
        bones.append((node, end))
    cmds.currentTime(0)
    for node, end in bones:
        suite['position'](end)
    start = time.perf_counter()
    for frame in range(1, 21):
        cmds.currentTime(frame)
        for node, end in bones:
            suite['position'](end)
    elapsed = time.perf_counter() - start
    builds = cmds.getAttr(collider + '.geometryBuilds')
    if not deforming and builds != 1:
        raise AssertionError('Static mesh accelerator was rebuilt')
    results.append({'deforming': deforming, 'root_overlap_fallback': fallback,
                    'triangles': subdivisions * subdivisions * 2,
                    'bones': count, 'capsules': 5, 'frames': 20,
                    'elapsed_seconds': round(elapsed, 4), 'geometry_builds': builds,
                    'milliseconds_per_frame': round(elapsed * 1000 / 20, 3)})
print(json.dumps(results, indent=2))
suite['maya'].standalone.uninitialize()
