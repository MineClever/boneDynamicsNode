"""Run with mayapy tests/maya_regression.py <absolute path to rebuilt .mll>.

Batch checks cover solver behavior and animated bounds. Viewport wire drawing
still requires the interactive check in sample_scripts/animated_colliders.py.
"""
import math
import os
import sys
import unittest

import maya.standalone
maya.standalone.initialize(name="python")
from maya import cmds


PLUGIN = os.path.abspath(sys.argv.pop(1))
cmds.loadPlugin(PLUGIN)


def make_solver(scale):
    node = cmds.createNode("boneDynamicsNode")
    cmds.setAttr(node + ".endTranslate", 10, 0, 0)
    cmds.setAttr(node + ".gravityMultiply", 1)
    cmds.setAttr(node + ".timeScale", scale)
    cmds.connectAttr("time1.outTime", node + ".time")
    target = cmds.createNode("transform")
    cmds.connectAttr(node + ".outputRotate", target + ".rotate")
    return node, target


def simulate(scale, frames, mode="off"):
    cmds.file(new=True, force=True)
    cmds.currentUnit(time="ntsc")
    cmds.evaluationManager(mode=mode)
    node, target = make_solver(scale)
    samples = []
    for frame in range(frames + 1):
        cmds.currentTime(frame)
        samples.append(cmds.getAttr(target + ".rotate")[0])
    return samples


class DynamicsRegression(unittest.TestCase):
    def test_acceleration_integrates_the_full_duration(self):
        for mode in ("off", "serial", "parallel"):
            for scale in (2, 3.5, 10, 100):
                # Compare equal simulation durations with identical static inputs.
                accelerated = simulate(scale, 2, mode)[-1]
                reference = simulate(1, int(scale * 2), mode)[-1]
                self.assertGreater(max(abs(v) for v in accelerated), 0.001)
                for actual, expected in zip(accelerated, reference):
                    self.assertAlmostEqual(actual, expected, delta=0.15,
                                           msg=(mode, scale, accelerated, reference))

    def test_small_and_large_scales_remain_finite(self):
        for scale in (0.01, 0.1, 1, 1.5, 2, 10, 100, 1000):
            samples = simulate(scale, 4)
            self.assertTrue(all(math.isfinite(v) for row in samples for v in row))
            self.assertGreater(max(abs(v) for v in samples[-1]), 1e-8)

    def test_pause_does_not_accumulate_time(self):
        reference = simulate(1, 2)[-1]
        cmds.file(new=True, force=True)
        cmds.currentUnit(time="ntsc")
        node, target = make_solver(1)
        for frame in (0, 1):
            cmds.currentTime(frame)
            cmds.getAttr(target + ".rotate")
        cmds.setAttr(node + ".timeScale", 0)
        for frame in range(2, 20):
            cmds.currentTime(frame)
            cmds.getAttr(target + ".rotate")
        cmds.setAttr(node + ".timeScale", 1)
        cmds.currentTime(20)
        actual = cmds.getAttr(target + ".rotate")[0]
        for a, b in zip(actual, reference):
            self.assertAlmostEqual(a, b, delta=0.001)

    def test_offset_is_applied_once_per_scene_frame(self):
        rotations = []
        for scale in (0.125, 1, 10):
            cmds.file(new=True, force=True)
            cmds.currentUnit(time="ntsc")
            node, target = make_solver(scale)
            cmds.setAttr(node + ".gravityMultiply", 0)
            cmds.setAttr(node + ".elasticity", 0)
            cmds.setAttr(node + ".damping", 1e9)
            cmds.currentTime(0)
            cmds.getAttr(target + ".rotate")
            cmds.setAttr(node + ".offsetMatrix",
                         1, 0, 0, 0, 0, 1, 0, 0,
                         0, 0, 1, 0, 0, 1, 0, 1, type="matrix")
            cmds.currentTime(1)
            rotations.append(cmds.getAttr(target + ".rotate")[0])
        self.assertGreater(abs(rotations[0][2]), 1)
        for actual in rotations[1:]:
            for a, b in zip(actual, rotations[0]):
                self.assertAlmostEqual(a, b, delta=1e-6)

    def test_animated_collider_bounds(self):
        import maya.api.OpenMaya as om
        for mode in ("off", "serial", "parallel"):
            cmds.file(new=True, force=True)
            cmds.currentUnit(linear="cm")
            cmds.evaluationManager(mode=mode)
            for kind, attr in (("sphereColliderNode", "radius"),
                               ("capsuleColliderNode", "radiusA"),
                               ("capsuleColliderNode", "radiusB"),
                               ("capsuleColliderNode", "height")):
                shape = cmds.createNode(kind)
                for frame, value in ((1, 1), (10, 5)):
                    cmds.setKeyframe(shape, attribute=attr, time=frame, value=value)
                selection = om.MSelectionList()
                selection.add(shape)
                fn = om.MFnDagNode(selection.getDependNode(0))
                sizes = []
                for frame in (1, 5, 10, 5, 1):
                    cmds.currentTime(frame)
                    bounds = fn.boundingBox
                    sizes.append((bounds.width, bounds.height, bounds.depth))
                self.assertGreater(sizes[2][1], sizes[0][1])
                self.assertEqual(sizes[0], sizes[-1])
                self.assertEqual(sizes[1], sizes[-2])


if __name__ == "__main__":
    program = unittest.main(verbosity=2, exit=False)
    maya.standalone.uninitialize()
    sys.exit(0 if program.result.wasSuccessful() else 1)
