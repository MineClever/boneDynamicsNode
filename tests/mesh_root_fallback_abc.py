"""Optional real-asset acceptance: mayapy this.py <mll> <asset-dir> <output.json>.

Reads MagicBoneDynamic's public model from the sibling repository, without
loading its UI/test harness or writing its sources, pluginLoader or input files.
"""
import hashlib
import json
import math
import os
import sys
import time

sys.dont_write_bytecode = True
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
plugin, assets, output = map(os.path.abspath, sys.argv[1:4])
os.environ['MAYA_SKIP_USERSETUP_PY'] = '1'
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(1, os.path.join(os.path.dirname(ROOT), 'vaccineKiller', 'scripts'))
import maya.standalone
maya.standalone.initialize(name='python')
from maya import cmds
from maya.api import OpenMaya as om
cmds.loadPlugin(plugin)
cmds.loadPlugin('AbcImport', quiet=True)
from Menu_Script.Anim.Python.MagicBoneDynamic.model import MagicBoneDynamicModel


def digest(path):
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


inputs = [os.path.join(assets, name) for name in
          ('scene_spring_magic.ma', 'bound_preset_01.mbd.json', 'lowbody_anim_cache_01.abc')]
hashes = {path: digest(path) for path in inputs}
report = {'maya': cmds.about(version=True), 'api': cmds.about(apiVersion=True),
          'python': sys.version, 'plugin': cmds.pluginInfo('boneDynamicsNode', q=True, path=True),
          'sha256': digest(plugin), 'inputs': hashes, 'runs': []}
for enabled in (False, True):
    cmds.file(inputs[0], open=True, force=True)
    cmds.AbcImport(inputs[2], mode='import')
    mesh = '|Xiaoyu_Body_Collider|Xiaoyu_Body_ColliderShape'
    selected = om.MSelectionList()
    selected.add(mesh)
    fn = om.MFnMesh(selected.getDagPath(0))
    assert fn.numVertices == 695
    model = MagicBoneDynamicModel()
    chains = [item for item in model.presets.import_file(inputs[1], include_target_keys=False)
              if item.kind == 'chain']
    assert len(chains) == 8
    assert sum(len(item.members('solvers')) for item in chains) == 32
    model.collision.disconnect(chains)
    collider = model.collision.create('mesh', [mesh])
    model.collision.connect(chains, [collider])
    provider = collider.members('shape')[0]
    cmds.setAttr(provider + '.rootOverlapFallback', enabled)
    model.simulation.reset(chains)
    frames, previous = [], {}
    started = time.perf_counter()
    for frame in range(102):
        cmds.currentTime(frame, update=True)
        model.simulation.evaluate(chains)
        rows = []
        for item in chains:
            sims = item.members('sims')
            for index, node in enumerate(item.members('solvers')):
                root = om.MPoint(cmds.xform(sims[index], q=True, ws=True, t=True))
                tip = om.MPoint(cmds.xform(sims[index+1], q=True, ws=True, t=True))
                q, _ = fn.getClosestPoint(tip, om.MSpace.kWorld)
                rq, _ = fn.getClosestPoint(root, om.MSpace.kWorld)
                radius = cmds.getAttr(node + '.radius')
                offset = cmds.getAttr(provider + '.contactOffset')
                key = (item.label, index)
                coords = list(tip)[:3]
                assert all(math.isfinite(v) for v in coords)
                active = cmds.getAttr(node + '.meshRootFallbackActive')
                before = previous.get(key)
                rows.append({'chain': item.label, 'segment': index,
                             'tip': coords, 'length': (tip-root).length(),
                             'root_gap': (root-rq).length()-cmds.getAttr(node+'.rootRadius')-offset,
                             'tip_gap': (tip-q).length()-radius-offset,
                             'fallback': active, 'unresolved': cmds.getAttr(node+'.meshCollisionUnresolved'),
                             'recovered': bool(before and before[0] and not active),
                             'tip_displacement': (tip-before[1]).length() if before else 0})
                previous[key] = active, tip
        frames.append({'frame': frame, 'rows': rows})
    run = {'enabled': enabled, 'seconds': time.perf_counter()-started, 'frames': frames}
    report['runs'].append(run)
    selected_rows = [r for r in frames[10]['rows'] if r['segment'] == 0 and 'Skirt' in r['chain']]
    print(json.dumps({'enabled': enabled, 'seconds': run['seconds'], 'frame10_skirt': selected_rows}, sort_keys=True))
assert hashes == {path: digest(path) for path in inputs}, 'Input assets changed'
for run in report['runs']:
    for frame in run['frames'][1:]:
        for row in frame['rows']:
            assert row['tip_gap'] >= -1e-4 or row['unresolved'], 'Residual penetration was hidden'
for before, after in zip(report['runs'][0]['frames'], report['runs'][1]['frames']):
    for a, b in zip(before['rows'], after['rows']):
        assert abs(a['length']-b['length']) < 1e-5, 'Fallback changed bone length'
with open(output, 'w') as stream:
    json.dump(report, stream, indent=2, sort_keys=True)
print('REPORT ' + output)
maya.standalone.uninitialize()
