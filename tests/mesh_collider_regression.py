"""mayapy tests/mesh_collider_regression.py <rebuilt boneDynamicsNode.mll>."""
import math
import os
import sys
import tempfile
import unittest

import maya.standalone
maya.standalone.initialize(name='python')
from maya import cmds

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
cmds.loadPlugin(os.path.abspath(sys.argv.pop(1)))
import bdn


def bone(root=(0, 2, 0), length=10, radius=0.1, capsule=False, simulation_scale=1):
    node = cmds.createNode('boneDynamicsNode')
    cmds.setAttr(node + '.boneTranslate', *root)
    cmds.setAttr(node + '.endTranslate', length, 0, 0)
    cmds.setAttr(node + '.radius', radius)
    cmds.setAttr(node + '.rootRadius', radius)
    cmds.setAttr(node + '.boneAsCapsule', capsule)
    cmds.setAttr(node + '.elasticity', 0)
    cmds.setAttr(node + '.damping', 0)
    cmds.setAttr(node + '.simulationScale', simulation_scale)
    cmds.connectAttr('time1.outTime', node + '.time')
    # Represent the actual solved tip in world space for geometric assertions.
    joint = cmds.createNode('joint')
    cmds.connectAttr(node + '.boneTranslate', joint + '.translate')
    end = cmds.createNode('transform', parent=joint)
    cmds.setAttr(end + '.translateX', length)
    cmds.connectAttr(node + '.outputRotate', joint + '.rotate')
    return node, end


def position(end):
    return cmds.xform(end, query=True, worldSpace=True, translation=True)


def evaluate(end, frame):
    cmds.currentTime(frame)
    value = position(end)
    if not all(math.isfinite(v) for v in value):
        raise AssertionError('Nonfinite simulation result: {}'.format(value))
    return value


def plane(y=0, x=0, width=100, subdivisions=1):
    mesh = cmds.polyPlane(width=width, height=width, subdivisionsX=subdivisions,
                          subdivisionsY=subdivisions, axis=(0, 1, 0))[0]
    cmds.setAttr(mesh + '.translate', x, y, 0)
    return mesh


def connect(node, mesh, **settings):
    collider = bdn.create_mesh_collider(mesh, **settings)
    cmds.connectAttr(collider + '.outputCollider', node + '.meshColliderData[0]')
    return collider


class MeshColliderRegression(unittest.TestCase):
    def setUp(self):
        cmds.file(new=True, force=True)
        cmds.currentUnit(time='ntsc', linear='cm')
        cmds.evaluationManager(mode='off')

    def test_high_speed_sphere_does_not_cross_thin_plane(self):
        for mode in ('off', 'serial', 'parallel'):
            for unit in ('film', 'pal', '120fps'):
                with self.subTest(mode=mode, unit=unit):
                    self.setUp()
                    cmds.evaluationManager(mode=mode)
                    cmds.currentUnit(time=unit)
                    node, end = bone()
                    connect(node, plane())
                    cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
                    evaluate(end, 0)
                    actual = evaluate(end, 1)
                    self.assertGreaterEqual(actual[1], 0.099)
                    self.assertAlmostEqual(sum((a-b)**2 for a, b in zip(actual, (0, 2, 0))), 100, delta=1e-6)

    def test_moving_mesh_pushes_stationary_sphere(self):
        for mode in ('off', 'serial', 'parallel'):
            with self.subTest(mode=mode):
                self.setUp()
                cmds.evaluationManager(mode=mode)
                mesh = plane(-2)
                for frame, value in ((0, -2), (1, 4)):
                    cmds.setKeyframe(mesh, attribute='translateY', time=frame, value=value)
                cmds.keyTangent(mesh, attribute='translateY', inTangentType='linear', outTangentType='linear')
                node, end = bone()
                collider = connect(node, mesh)
                evaluate(end, 0)
                actual = evaluate(end, 1)
                self.assertGreaterEqual(actual[1], 4.099)
                self.assertEqual(cmds.getAttr(collider + '.sampleCount'), 5)
                self.assertEqual(cmds.currentTime(query=True), 1)

    def test_subframe_out_and_back_motion_is_sampled(self):
        results = []
        for samples in (1, 4):
            self.setUp()
            mesh = plane(-2)
            for frame, value in ((0, -2), (0.5, 4), (1, -2)):
                cmds.setKeyframe(mesh, attribute='translateY', time=frame, value=value)
            cmds.keyTangent(mesh, attribute='translateY', inTangentType='linear', outTangentType='linear')
            node, end = bone()
            connect(node, mesh, time_samples=samples)
            evaluate(end, 0)
            results.append(evaluate(end, 1)[1])
        self.assertAlmostEqual(results[0], 2, delta=1e-5)
        self.assertGreater(results[1], 4)

    def test_skin_cluster_is_evaluated_at_time_samples(self):
        for mode in ('off', 'serial', 'parallel'):
            with self.subTest(mode=mode):
                self.setUp()
                cmds.evaluationManager(mode=mode)
                mesh = plane(-2)
                cmds.select(clear=True)
                joint = cmds.joint(position=(0, 0, 0))
                cmds.skinCluster(joint, mesh, toSelectedBones=True)
                for frame, value in ((0, 0), (1, 6)):
                    cmds.setKeyframe(joint, attribute='translateY', time=frame, value=value)
                cmds.keyTangent(joint, attribute='translateY', inTangentType='linear', outTangentType='linear')
                node, end = bone()
                collider = connect(node, mesh)
                evaluate(end, 0)
                actual = evaluate(end, 1)
                self.assertGreaterEqual(actual[1], 4.099)
                self.assertGreater(cmds.getAttr(collider + '.geometryBuilds'), 1)

    def test_blendshape_deformation_is_sampled(self):
        mesh = plane(-2)
        target = cmds.duplicate(mesh)[0]
        cmds.move(0, 6, 0, target + '.vtx[*]', relative=True)
        blend = cmds.blendShape(target, mesh)[0]
        cmds.delete(target)
        for frame, value in ((0, 0), (1, 1)):
            cmds.setKeyframe(blend + '.weight[0]', time=frame, value=value)
        node, end = bone()
        connect(node, mesh)
        evaluate(end, 0)
        self.assertGreaterEqual(evaluate(end, 1)[1], 4.099)

    def test_capsule_midsection_hits_when_tip_misses(self):
        results = []
        for capsule in (False, True):
            self.setUp()
            mesh = plane(0.9, x=5, width=2)
            node, end = bone(root=(0, 1, 0), radius=0.3, capsule=capsule)
            connect(node, mesh)
            evaluate(end, 0)
            tip = evaluate(end, 1)
            results.append(tip[1])
            if capsule:
                for i in range(1, 1000):
                    t = i / 1000.0
                    x = tip[0] * t
                    if 4 <= x <= 6:
                        self.assertGreaterEqual(1 + (tip[1] - 1) * t - 0.9, 0.299)
        self.assertAlmostEqual(results[0], 1, delta=1e-5)
        self.assertGreater(results[1], 1.2)

    def test_capsule_sweeps_against_moving_triangle(self):
        mesh = plane(2, x=5, width=2)
        for frame, value in ((0, 2), (1, -2)):
            cmds.setKeyframe(mesh, attribute='translateY', time=frame, value=value)
        cmds.keyTangent(mesh, attribute='translateY', inTangentType='linear', outTangentType='linear')
        node, end = bone(root=(0, 0, 0), radius=0.2, capsule=True)
        connect(node, mesh)
        evaluate(end, 0)
        self.assertLess(evaluate(end, 1)[1], -0.2)

    def test_named_vertex_color_set_excludes_faces(self):
        for masked in (False, True):
            with self.subTest(masked=masked):
                self.setUp()
                mesh = plane()
                cmds.polyColorSet(mesh, create=True, colorSet='customExclude')
                cmds.polyColorPerVertex(mesh + '.vtx[*]', rgb=(1, 0, 0))
                node, end = bone()
                connect(node, mesh, exclude_color_set='customExclude' if masked else 'notPresent')
                cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
                evaluate(end, 0)
                y = evaluate(end, 1)[1]
                self.assertLess(y, 0) if masked else self.assertGreaterEqual(y, 0.099)

    def test_static_cache_shared_and_invalidated_by_edit(self):
        mesh = plane()
        collider = bdn.create_mesh_collider(mesh)
        self.assertEqual(bdn.create_mesh_collider(mesh), collider)
        bones = [bone() for _ in range(5)]
        for node, end in bones:
            bdn.connect_colliders(node, [collider])
        for frame in range(4):
            for node, end in bones:
                evaluate(end, frame)
        self.assertEqual(cmds.getAttr(collider + '.geometryBuilds'), 1)
        cmds.move(0, 0.5, 0, mesh + '.vtx[0]', relative=True)
        evaluate(bones[0][1], 4)
        self.assertGreater(cmds.getAttr(collider + '.geometryBuilds'), 1)

    def test_simulation_scale_and_time_scale(self):
        for scale, simulation_scale in ((1, 1), (1000, 0.001)):
            with self.subTest(scale=scale):
                self.setUp()
                node, end = bone(root=(0, 2 * scale, 0), length=10 * scale,
                                 radius=0.1 * scale, simulation_scale=simulation_scale)
                cmds.setAttr(node + '.timeScale', 10)
                connect(node, plane(width=100 * scale), contact_offset=0.001 * scale)
                cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
                evaluate(end, 0)
                y = evaluate(end, 1)[1] / scale
                self.assertGreaterEqual(y, 0.099)

    def test_connections_and_color_settings_survive_save_load(self):
        mesh = plane()
        node, end = bone()
        collider = connect(node, mesh, exclude_color_set='persistedMask', time_samples=8)
        directory = os.path.join(ROOT, 'build')
        with tempfile.TemporaryDirectory(dir=directory) as temporary:
            scene = os.path.join(temporary, 'collider.ma')
            cmds.file(rename=scene)
            cmds.file(save=True, force=True, type='mayaAscii')
            cmds.file(new=True, force=True)
            cmds.file(scene, open=True, force=True)
            self.assertEqual(cmds.getAttr(collider + '.excludeColorSet'), 'persistedMask')
            self.assertEqual(cmds.getAttr(collider + '.timeSamples'), 8)
            self.assertTrue(cmds.isConnected(collider + '.outputCollider', node + '.meshColliderData[0]'))
            cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
            evaluate(end, 0)
            self.assertGreaterEqual(evaluate(end, 1)[1], 0.099)

    def test_live_mask_and_partial_face_colors(self):
        for fully_red in (False, True):
            with self.subTest(fully_red=fully_red):
                self.setUp()
                mesh = plane()
                cmds.polyColorSet(mesh, create=True, colorSet='bdnCollisionExclude')
                cmds.polyColorPerVertex(mesh + '.vtx[*]', rgb=(0, 0, 0))
                node, end = bone()
                collider = connect(node, mesh)
                evaluate(end, 0)
                cmds.polyColorPerVertex(mesh + ('.vtx[*]' if fully_red else '.vtx[0]'), rgb=(1, 0, 0))
                # Reevaluate edited geometry at the reset frame before advancing.
                evaluate(end, 0)
                cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
                y = evaluate(end, 1)[1]
                self.assertLess(y, 0) if fully_red else self.assertGreaterEqual(y, 0.099)

    def test_unassigned_colors_remain_collidable_at_zero_threshold(self):
        mesh = plane()
        cmds.polyColorSet(mesh, create=True, colorSet='bdnCollisionExclude')
        node, end = bone()
        collider = connect(node, mesh)
        cmds.setAttr(collider + '.excludeThreshold', 0)
        cmds.setAttr(node + '.additionalForceY', -1e8)
        evaluate(end, 0)
        self.assertGreaterEqual(evaluate(end, 1)[1], 0.099)

    def test_topology_changes_report_lost_ccd_history(self):
        mesh = plane()
        node, end = bone()
        collider = connect(node, mesh)
        evaluate(end, 0)
        generator = cmds.ls(type='polyPlane')[0]
        cmds.setAttr(generator + '.subdivisionsWidth', 3)
        evaluate(end, 1)
        self.assertTrue(cmds.getAttr(collider + '.topologyChanged'))
        self.assertTrue(cmds.getAttr(node + '.meshCollisionUnresolved'))
        evaluate(end, 2)
        self.assertFalse(cmds.getAttr(collider + '.topologyChanged'))

    def test_sparse_multiple_colliders_and_disconnect(self):
        node, end = bone()
        a = bdn.create_mesh_collider(plane())
        b = bdn.create_mesh_collider(plane(-5))
        cmds.connectAttr(a + '.outputCollider', node + '.meshColliderData[7]')
        cmds.connectAttr(b + '.outputCollider', node + '.meshColliderData[19]')
        cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
        evaluate(end, 0)
        self.assertGreaterEqual(evaluate(end, 1)[1], 0.099)
        bdn.disconnect_colliders(node)
        evaluate(end, 0)
        self.assertLess(evaluate(end, 1)[1], 0)

    def test_mesh_dag_instance_uses_selected_transform(self):
        mesh = plane(-20)
        instance = cmds.instance(mesh)[0]
        cmds.setAttr(instance + '.translateY', 0)
        node, end = bone()
        connect(node, instance)
        cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
        evaluate(end, 0)
        self.assertGreaterEqual(evaluate(end, 1)[1], 0.099)

    def test_disabling_ccd_allows_the_high_speed_crossing(self):
        node, end = bone()
        connect(node, plane(), continuous=False)
        cmds.setAttr(node + '.additionalForce', 0, -1e8, 0)
        evaluate(end, 0)
        self.assertLess(evaluate(end, 1)[1], 0)

    def test_anchored_root_infeasibility_is_reported(self):
        mesh = plane(0.1, width=100)
        node, end = bone(root=(0, 0, 0), radius=0.3, capsule=True)
        connect(node, mesh)
        evaluate(end, 0)
        evaluate(end, 1)
        self.assertTrue(cmds.getAttr(node + '.meshCollisionUnresolved'))

    def test_animated_offset_does_not_teleport_through_mesh(self):
        node, end = bone()
        connect(node, plane())
        offset = cmds.createNode('transform')
        cmds.connectAttr(offset + '.worldMatrix[0]', node + '.offsetMatrix')
        for frame, delta in ((0, 0), (1, -4)):
            cmds.setKeyframe(node, attribute='boneTranslateY', time=frame, value=2 + delta)
            cmds.setKeyframe(offset, attribute='translateY', time=frame, value=delta)
        evaluate(end, 0)
        self.assertGreaterEqual(evaluate(end, 1)[1], 0.099)

    def test_selected_face_hole_does_not_disable_neighbor(self):
        from maya.api import OpenMaya as om
        mesh = plane(y=1.5, width=40, subdivisions=2)
        cmds.polyColorSet(mesh, create=True, colorSet='bdnCollisionExclude')
        cmds.polyColorPerVertex(mesh + '.vtx[*]', rgb=(0, 0, 0))
        shape = cmds.listRelatives(mesh, shapes=True)[0]
        selection = om.MSelectionList(); selection.add(shape)
        fn = om.MFnMesh(selection.getDagPath(0))
        # Pick a single quadrant, and paint face-vertex colors so shared vertices
        # do not accidentally remove adjoining faces.
        chosen = 0
        ids = fn.getPolygonVertices(chosen)
        center = [sum(fn.getPoint(v)[i] for v in ids) / len(ids) for i in range(3)]
        cmds.polyColorPerVertex([mesh + '.vtxFace[{}][{}]'.format(v, chosen) for v in ids], rgb=(1, 0, 0))
        provider = bdn.create_mesh_collider(mesh)
        ends = []
        for x, z in ((center[0], center[2]), (-center[0], -center[2])):
            node, end = bone(root=(x - 1, 2, z), length=1)
            bdn.connect_colliders(node, [provider])
            cmds.setAttr(node + '.additionalForceY', -1e8)
            ends.append(end)
        for end in ends: evaluate(end, 0)
        values = [evaluate(end, 1)[1] for end in ends]
        self.assertLess(values[0], 1.5)
        self.assertGreaterEqual(values[1], 1.599)


if __name__ == '__main__':
    program = unittest.main(verbosity=2, exit=False)
    maya.standalone.uninitialize()
    sys.exit(0 if program.result.wasSuccessful() else 1)
