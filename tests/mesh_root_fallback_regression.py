"""mayapy tests/mesh_root_fallback_regression.py <rebuilt mll>."""
import math
import os
import tempfile
import unittest

from mesh_collider_regression import cmds, bdn, bone, plane, connect, evaluate


class RootFallbackRegression(unittest.TestCase):
    def setUp(self):
        cmds.file(new=True, force=True)
        cmds.currentUnit(time='ntsc', linear='cm')
        cmds.evaluationManager(mode='off')

    def fixture(self, enabled=True, **settings):
        node, end = bone(root=(0, .02, 0), radius=.05, capsule=True,
                         simulation_scale=settings.pop('simulation_scale', 1))
        provider = connect(node, plane(), **settings)
        if enabled:
            cmds.setAttr(provider + '.rootOverlapFallback', True)
        return node, end, provider

    def check_tip(self, value, root=(0, .02, 0), length=10):
        self.assertTrue(all(math.isfinite(v) for v in value))
        self.assertAlmostEqual(sum((v-r)**2 for v, r in zip(value, root)), length**2, delta=1e-5)
        self.assertGreaterEqual(abs(value[1]), .05099)

    def test_original_root_conflict(self):
        node, end, provider = self.fixture(False)
        evaluate(end, 0)
        evaluate(end, 1)
        self.assertTrue(cmds.getAttr(node + '.meshCollisionUnresolved'))

    def test_feasible_endpoint_and_default(self):
        self.assertFalse(cmds.getAttr(cmds.createNode('meshColliderNode') + '.rootOverlapFallback'))
        node, end, provider = self.fixture()
        evaluate(end, 0)
        for frame in range(1, 8):
            self.check_tip(evaluate(end, frame))
            self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
            self.assertFalse(cmds.getAttr(node + '.meshCollisionUnresolved'))
            self.assertTrue(cmds.getAttr(node + '.boneAsCapsule'))

    def test_shared_provider_sparse_and_consumer_order(self):
        node, end, provider = self.fixture()
        other, other_end = bone(root=(0, 2, 0), capsule=True)
        cmds.connectAttr(provider + '.outputCollider', other + '.meshColliderData[17]')
        for reverse in (False, True):
            cmds.currentTime(0)
            cmds.xform(end, query=True, worldSpace=True, translation=True)
            cmds.xform(other_end, query=True, worldSpace=True, translation=True)
            for frame in range(1, 5):
                cmds.currentTime(frame)
                for tip in ((end, other_end) if reverse else (other_end, end)):
                    value = cmds.xform(tip, query=True, worldSpace=True, translation=True)
                    if tip == end:
                        self.check_tip(value)
                self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
                self.assertFalse(cmds.getAttr(other + '.meshRootFallbackActive'))

    def test_scale_time_and_evaluation_modes(self):
        for mode in ('off', 'serial', 'parallel'):
            for scale in (.01, 1, 100):
                self.setUp()
                cmds.evaluationManager(mode=mode)
                node, end, provider = self.fixture(simulation_scale=scale)
                cmds.setAttr(node + '.timeScale', {.01: .1, 1: 1, 100: 10}[scale])
                evaluate(end, 0)
                self.check_tip(evaluate(end, 1))
                self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))

    def test_python_compatibility_reuse_and_cache(self):
        mesh = plane()
        legacy = bdn.create_mesh_collider(mesh, 'bdnCollisionExclude', 4, .001, True, True, None)
        enabled = bdn.create_mesh_collider(mesh, root_overlap_fallback=True)
        self.assertNotEqual(legacy, enabled)
        self.assertEqual(enabled, bdn.create_mesh_collider(mesh, root_overlap_fallback=True))
        node, end = bone(root=(0, .02, 0), radius=.05, capsule=True)
        cmds.connectAttr(enabled + '.outputCollider', node + '.meshColliderData[3]')
        evaluate(end, 0)
        for frame in range(1, 5):
            evaluate(end, frame)
        self.assertEqual(cmds.getAttr(enabled + '.geometryBuilds'), 1)

    def test_recovery_and_hysteresis(self):
        node, end, provider = self.fixture()
        # Move the fixed root only; no collision-induced tip flip at recovery.
        values = ((0, .02), (1, .02), (2, .0515), (3, .051), (4, .06), (5, .08))
        for frame, y in values:
            cmds.setKeyframe(node, attribute='boneTranslateY', time=frame, value=y)
        previous = None
        for frame, y in values:
            value = evaluate(end, frame)
            if previous:
                # Less than 3 degrees on a length-10 segment, including motion
                # carried by the ordinary undamped integrator after contact.
                self.assertLess(math.sqrt(sum((a-b)**2 for a, b in zip(value, previous))), .5)
            previous = value
            if frame in (1, 2, 3):
                self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
            if frame == 5:
                self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))
                self.assertFalse(cmds.getAttr(node + '.meshCollisionUnresolved'))

    def test_reset_disconnect_and_dynamic_switch(self):
        node, end, provider = self.fixture()
        evaluate(end, 0)
        evaluate(end, 1)
        self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
        evaluate(end, 0)
        self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))
        evaluate(end, 1)
        cmds.disconnectAttr(provider + '.outputCollider', node + '.meshColliderData[0]')
        evaluate(end, 2)
        self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))
        cmds.connectAttr(provider + '.outputCollider', node + '.meshColliderData[0]')
        cmds.setAttr(provider + '.rootOverlapFallback', False)
        evaluate(end, 3)
        self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))
        self.assertTrue(cmds.getAttr(node + '.meshCollisionUnresolved'))

    def test_endpoint_infeasible_remains_unresolved(self):
        node, end = bone(root=(0, .02, 0), length=.01, radius=.05, capsule=True)
        connect(node, plane(), root_overlap_fallback=True)
        evaluate(end, 0)
        evaluate(end, 1)
        self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
        self.assertTrue(cmds.getAttr(node + '.meshCollisionUnresolved'))

    def test_no_overlap_retains_midsection_collision(self):
        mesh = plane(width=2, x=5)
        node, end = bone(root=(0, .02, 0), radius=.05, capsule=True)
        connect(node, mesh, root_overlap_fallback=True)
        evaluate(end, 0)
        value = evaluate(end, 1)
        self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))
        self.assertGreater(abs(value[1] - .02), .01)
        self.assertAlmostEqual(sum((v-r)**2 for v, r in zip(value, (0, .02, 0))), 100, delta=1e-5)

    def test_all_excluded_and_degenerate_do_not_trigger(self):
        for degenerate in (False, True):
            self.setUp()
            mesh = plane()
            if degenerate:
                cmds.scale(1, 1, 0, mesh + '.vtx[*]', pivot=(0, 0, 0), absolute=True)
            else:
                cmds.polyColorSet(mesh, create=True, colorSet='bdnCollisionExclude')
                cmds.polyColorPerVertex(mesh + '.vtx[*]', rgb=(1, 0, 0))
            node, end = bone(root=(0, .02, 0), radius=.05, capsule=True)
            connect(node, mesh, root_overlap_fallback=True)
            evaluate(end, 0)
            value = evaluate(end, 1)
            self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'), 'degenerate={}'.format(degenerate))
            self.assertFalse(cmds.getAttr(node + '.meshCollisionUnresolved'))
            self.assertAlmostEqual(value[1], .02, places=6)

    def test_swept_root_and_moving_deformer_subframes(self):
        for deform in (False, True):
            for continuous in (False, True):
                for samples in (1, 4):
                    self.setUp()
                    mesh = plane(y=-1)
                    target = cmds.cluster(mesh + '.vtx[*]')[1] if deform else mesh
                    for frame, y in ((0, 0 if deform else -1), (1, 2 if deform else 1)):
                        cmds.setKeyframe(target, attribute='translateY', time=frame, value=y)
                    cmds.keyTangent(target, inTangentType='linear', outTangentType='linear')
                    node, end = bone(root=(0, 0, 0), radius=.05, capsule=True)
                    connect(node, mesh, root_overlap_fallback=True, continuous=continuous, time_samples=samples)
                    evaluate(end, 0)
                    evaluate(end, .5)
                    self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
                    evaluate(end, .75)
                    self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
                    evaluate(end, 1)
                    self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))

    def test_save_reload_undo_and_replay(self):
        node, end, provider = self.fixture()
        cmds.undoInfo(state=True)
        cmds.setAttr(provider + '.rootOverlapFallback', False)
        cmds.undo()
        self.assertTrue(cmds.getAttr(provider + '.rootOverlapFallback'))
        cmds.redo()
        self.assertFalse(cmds.getAttr(provider + '.rootOverlapFallback'))
        cmds.setAttr(provider + '.rootOverlapFallback', True)
        path = os.path.join(tempfile.gettempdir(), 'bdn_root_fallback_test.ma')
        cmds.file(rename=path)
        cmds.file(save=True, type='mayaAscii', force=True)
        try:
            evaluate(end, 0)
            first = [evaluate(end, frame) for frame in range(1, 5)]
            cmds.file(path, open=True, force=True)
            self.assertTrue(cmds.getAttr(provider + '.rootOverlapFallback'))
            evaluate(end, 0)
            replay = [evaluate(end, frame) for frame in range(1, 5)]
            for a, b in zip(first, replay):
                for x, y in zip(a, b):
                    self.assertAlmostEqual(x, y, places=7)
        finally:
            os.remove(path)

    def test_context_query_does_not_consume_playback(self):
        from maya.api import OpenMaya as om
        node, end, provider = self.fixture()
        evaluate(end, 0)
        evaluate(end, 1)
        selection = om.MSelectionList()
        selection.add(node + '.outputRotateX')
        plug = selection.getPlug(0)
        plug.asMAngle(om.MDGContext(om.MTime(1.5, om.MTime.uiUnit())))
        actual = evaluate(end, 2)
        evaluate(end, 0)
        evaluate(end, 1)
        expected = evaluate(end, 2)
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def test_multiple_providers_keep_their_own_modes(self):
        node, end, provider = self.fixture()
        # This provider must retain capsule checks, even though another provider
        # in the same substep has explicitly fallen back.
        middle = bdn.create_mesh_collider(plane(y=.03, x=5, width=2), root_overlap_fallback=False)
        cmds.connectAttr(middle + '.outputCollider', node + '.meshColliderData[21]')
        evaluate(end, 0)
        value = evaluate(end, 1)
        self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
        self.assertGreater(abs(value[1] - .02), .07)
        self.assertAlmostEqual(sum((v-r)**2 for v, r in zip(value, (0, .02, 0))), 100, delta=1e-5)

    def test_root_trajectory_crosses_between_frame_endpoints(self):
        node, end = bone(root=(0, -1, 0), radius=.05, capsule=True)
        connect(node, plane(), root_overlap_fallback=True)
        cmds.setKeyframe(node, attribute='boneTranslateY', time=0, value=-1)
        cmds.setKeyframe(node, attribute='boneTranslateY', time=1, value=1)
        cmds.keyTangent(node, inTangentType='linear', outTangentType='linear')
        evaluate(end, 0)
        evaluate(end, 1)
        self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
        # Full-interval diagnostic records entry; a subsequent clear interval
        # restores capsule mode without touching the animated root.
        value = evaluate(end, 2)
        self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))
        self.assertAlmostEqual(sum((v-r)**2 for v, r in zip(value, (0, 1, 0))), 100, delta=1e-5)

    def test_instance_parent_transform_and_large_world(self):
        for scale in (1, 10000):
            self.setUp()
            original = plane(y=-10, width=100*scale)
            instance = cmds.instance(original)[0]
            parent = cmds.createNode('transform')
            cmds.parent(instance, parent)
            cmds.setAttr(parent + '.scale', 2, 3, 4)
            cmds.setAttr(instance + '.translateY', 0)
            node, end = bone(root=(0, .02*scale, 0), length=10*scale,
                             radius=.05*scale, capsule=True, simulation_scale=1.0/scale)
            connect(node, instance, root_overlap_fallback=True, contact_offset=.001*scale)
            evaluate(end, 0)
            value = evaluate(end, 1)
            self.check_tip([v/scale for v in value])
            self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))

    def test_topology_source_rewire_and_gap_reset(self):
        node, end, provider = self.fixture()
        evaluate(end, 0)
        evaluate(end, 1)
        mesh = (cmds.listConnections(provider + '.inputMesh', source=True, shapes=True) or [])[0]
        generator = cmds.ls(cmds.listHistory(mesh), type='polyPlane')
        cmds.setAttr(generator[0] + '.subdivisionsWidth', 2)
        evaluate(end, 2)
        self.assertTrue(cmds.getAttr(node + '.meshCollisionUnresolved'))
        replacement = plane(y=-2)
        source = cmds.listRelatives(replacement, shapes=True, fullPath=True)[0] + '.worldMesh[0]'
        cmds.connectAttr(source, provider + '.inputMesh', force=True)
        evaluate(end, 3)
        self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))
        evaluate(end, 20)
        self.assertFalse(cmds.getAttr(node + '.meshRootFallbackActive'))

    def test_new_provider_reconnect_resolves_tip_immediately(self):
        node, end = bone(root=(0, .02, 0), radius=.05, capsule=True)
        evaluate(end, 0)
        evaluate(end, 1)
        provider = connect(node, plane(), root_overlap_fallback=True)
        value = evaluate(end, 2)
        self.check_tip(value)
        self.assertTrue(cmds.getAttr(node + '.meshRootFallbackActive'))
        self.assertFalse(cmds.getAttr(node + '.meshCollisionUnresolved'))

    def test_ae_templates_parse_and_expose_controls(self):
        from maya import mel
        directory = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts')
        for name, attribute in (('AEmeshColliderNodeTemplate.mel', 'rootOverlapFallback'),
                                ('AEboneDynamicsNodeTemplate.mel', 'meshRootFallbackActive')):
            path = os.path.join(directory, name).replace('\\', '/')
            mel.eval('source "{}";'.format(path))
            with open(path) as stream:
                self.assertIn('-addControl "{}"'.format(attribute), stream.read())


if __name__ == '__main__':
    unittest.main()
