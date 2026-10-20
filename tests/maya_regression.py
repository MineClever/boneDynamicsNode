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


def simulate(scale, frames, mode="off", unit="ntsc", turbulence=False, start_frame=0):
    cmds.file(new=True, force=True)
    cmds.currentUnit(time=unit)
    cmds.evaluationManager(mode=mode)
    node, target = make_solver(scale)
    cmds.setAttr(node + ".enableTurbulence", turbulence)
    if turbulence:
        cmds.setAttr(node + ".turbulenceStrength", 50)
    samples = []
    for frame in range(start_frame, start_frame + frames + 1):
        cmds.currentTime(frame)
        samples.append(cmds.getAttr(target + ".rotate")[0])
    return samples


def simulate_geometry(world_scale=1.0, simulation_scale=None, collision=None,
                      elastic_function=0, moving_offset=False, parent_transform=False,
                      bone_capsule=False, time_scale=1.0, mode="off"):
    """Geometrically similar scenes must match when solver sizes match."""
    cmds.file(new=True, force=True)
    cmds.currentUnit(time="ntsc", linear="cm")
    cmds.evaluationManager(mode=mode)
    node, target = make_solver(time_scale)
    if simulation_scale is not None:
        cmds.setAttr(node + ".simulationScale", simulation_scale)
    origin = (2, 3, -1)
    cmds.setAttr(node + ".boneTranslate", *(v * world_scale for v in origin))
    cmds.setAttr(node + ".endTranslate", 10 * world_scale, 0, 0)
    cmds.setAttr(node + ".radius", 0.3 * world_scale)
    cmds.setAttr(node + ".rootRadius", 0.2 * world_scale)
    cmds.setAttr(node + ".boneAsCapsule", bone_capsule)
    cmds.setAttr(node + ".elasticForceFunction", elastic_function)
    cmds.setAttr(node + ".enableTurbulence", True)
    cmds.setAttr(node + ".turbulenceStrength", 20)
    cmds.setAttr(node + ".additionalForce", 0, 0, 30)
    cmds.setAttr(node + ".enableAngleLimit", True)
    cmds.setAttr(node + ".angleLimit", 150)

    def matrix_at(offset):
        return (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                (origin[0] + offset[0]) * world_scale,
                (origin[1] + offset[1]) * world_scale,
                (origin[2] + offset[2]) * world_scale, 1)

    if collision == "sphere":
        cmds.setAttr(node + ".sphereCollider[0].sphereColMatrix",
                     *matrix_at((9, -1, 0)), type="matrix")
        cmds.setAttr(node + ".sphereCollider[0].sphereColRadius", 2 * world_scale)
    elif collision == "capsule":
        cmds.setAttr(node + ".capsuleCollider[0].capsuleColMatrixA",
                     *matrix_at((9, -3, 1)), type="matrix")
        cmds.setAttr(node + ".capsuleCollider[0].capsuleColMatrixB",
                     *matrix_at((9, 3, 1)), type="matrix")
        cmds.setAttr(node + ".capsuleCollider[0].capsuleColRadiusA", 1.8 * world_scale)
        cmds.setAttr(node + ".capsuleCollider[0].capsuleColRadiusB", 1.4 * world_scale)
    elif collision == "capsuleInput":
        cmds.setAttr(node + ".capsuleColliderInput[0].capsuleColliderMatrix",
                     *matrix_at((9, 0, 1)), type="matrix")
        cmds.setAttr(node + ".capsuleColliderInput[0].capsuleColliderHeight", 6 * world_scale)
        cmds.setAttr(node + ".capsuleColliderInput[0].capsuleColliderRadiusA", 1.4 * world_scale)
        cmds.setAttr(node + ".capsuleColliderInput[0].capsuleColliderRadiusB", 1.8 * world_scale)
    elif collision == "plane":
        cmds.setAttr(node + ".infinitePlaneCollider[0].infinitePlaneColMatrix",
                     *matrix_at((0, -1, 0)), type="matrix")
    elif collision == "ground":
        cmds.setAttr(node + ".enableGroundCol", True)
        cmds.setAttr(node + ".groundHeight", (origin[1] - 1) * world_scale)
    elif collision == "mesh":
        mesh = cmds.polyPlane(width=40 * world_scale, height=40 * world_scale,
                              subdivisionsX=1, subdivisionsY=1, axis=(0, 1, 0))[0]
        cmds.setAttr(mesh + ".translate", origin[0] * world_scale,
                     (origin[1] - 1) * world_scale, origin[2] * world_scale)
        shape = cmds.listRelatives(mesh, shapes=True)[0]
        cmds.connectAttr(shape + ".worldMesh[0]", node + ".meshCollider[0]")
        cmds.setAttr(node + ".meshColCutoff", 10 * world_scale)

    if parent_transform:
        parent = cmds.createNode("transform")
        cmds.setAttr(parent + ".translate", 50 * world_scale, 20 * world_scale, -10 * world_scale)
        cmds.setAttr(parent + ".rotate", 10, 20, 30)
        cmds.setAttr(parent + ".scale", 1.25, 0.8, 1.5)
        cmds.connectAttr(parent + ".worldMatrix[0]", node + ".boneParentMatrix")
        cmds.connectAttr(parent + ".worldInverseMatrix[0]", node + ".boneParentInverseMatrix")

    if moving_offset:
        offset = cmds.createNode("transform")
        cmds.connectAttr(offset + ".worldMatrix[0]", node + ".offsetMatrix")
        cmds.setAttr(node + ".offsetMatrixWeight", 0.65)

    samples = []
    for frame in range(13):
        if moving_offset:
            cmds.setAttr(offset + ".translate", 0.2 * frame * world_scale,
                         0.15 * frame * world_scale, -0.1 * frame * world_scale)
            cmds.setAttr(offset + ".rotateZ", frame * 2)
        cmds.currentTime(frame)
        samples.append(cmds.getAttr(target + ".rotate")[0])
    return samples


class DynamicsRegression(unittest.TestCase):
    def test_reverse_and_direction_changes_keep_integrating(self):
        for mode in ("off", "serial", "parallel"):
            for scale in (0.1, 1, 3.5):
                expected = simulate(scale, 6, mode=mode)
                for frames in ((20, 19, 18, 17, 16, 15, 14),
                               (20, 21, 22, 21, 20, 21, 20)):
                    cmds.file(new=True, force=True)
                    cmds.currentUnit(time="ntsc")
                    cmds.evaluationManager(mode=mode)
                    node, target = make_solver(scale)
                    actual = []
                    for frame in frames:
                        cmds.currentTime(frame)
                        actual.append(cmds.getAttr(target + ".rotate")[0])
                    self.assert_samples_equal(actual, expected)

    def test_large_jumps_reset_in_both_directions(self):
        for destination in (10, 30):
            cmds.file(new=True, force=True)
            cmds.currentUnit(time="ntsc")
            node, target = make_solver(1)
            for frame in (20, 21):
                cmds.currentTime(frame)
                previous = cmds.getAttr(target + ".rotate")[0]
            self.assertGreater(abs(previous[2]), 0.01)
            cmds.currentTime(destination)
            self.assertEqual(cmds.getAttr(target + ".rotate")[0], (0, 0, 0))

    def test_pause_and_resume_reverse_playback(self):
        expected = simulate(1, 2)[-1]
        cmds.file(new=True, force=True)
        cmds.currentUnit(time="ntsc")
        node, target = make_solver(1)
        for frame in (30, 29):
            cmds.currentTime(frame)
            paused = cmds.getAttr(target + ".rotate")[0]
        cmds.setAttr(node + ".timeScale", 0)
        for frame in range(28, 19, -1):
            cmds.currentTime(frame)
            self.assert_samples_equal(cmds.getAttr(target + ".rotate"), [paused])
        cmds.setAttr(node + ".timeScale", 1)
        cmds.currentTime(19)
        self.assert_samples_equal(cmds.getAttr(target + ".rotate"), [expected])

    def assert_samples_equal(self, actual, expected, tolerance=1e-6):
        for row_a, row_b in zip(actual, expected):
            for a, b in zip(row_a, row_b):
                self.assertTrue(math.isfinite(a))
                self.assertAlmostEqual(a, b, delta=tolerance)

    def test_simulation_scale_matches_smaller_geometry(self):
        for mode in ("off", "serial", "parallel"):
            for elastic_function in (0, 1, 2):
                for world_scale in (1000, 0.1):
                    with self.subTest(mode=mode, elastic=elastic_function, size=world_scale):
                        expected = simulate_geometry(elastic_function=elastic_function, mode=mode)
                        actual = simulate_geometry(world_scale, 1.0 / world_scale,
                                                   elastic_function=elastic_function, mode=mode)
                        self.assert_samples_equal(actual, expected)

    def test_simulation_scale_preserves_collision_geometry(self):
        for collision in ("sphere", "capsule", "capsuleInput", "plane", "ground", "mesh"):
            for bone_capsule in (False, True):
                with self.subTest(collision=collision, bone_capsule=bone_capsule):
                    expected = simulate_geometry(collision=collision, bone_capsule=bone_capsule)
                    free = simulate_geometry(bone_capsule=bone_capsule)
                    self.assertGreater(max(abs(a - b) for ra, rb in zip(expected, free)
                                           for a, b in zip(ra, rb)), 0.01)
                    for world_scale in (1000, 0.1):
                        actual = simulate_geometry(world_scale, 1.0 / world_scale,
                                                   collision=collision, bone_capsule=bone_capsule)
                        self.assert_samples_equal(actual, expected, tolerance=1e-4)

    def test_simulation_scale_with_offset_parent_and_time_scale(self):
        for time_scale in (0.1, 1, 3.5):
            expected = simulate_geometry(moving_offset=True, parent_transform=True,
                                         time_scale=time_scale)
            actual = simulate_geometry(1000, 0.001, moving_offset=True,
                                       parent_transform=True, time_scale=time_scale)
            self.assert_samples_equal(actual, expected)

    def test_simulation_scale_default_and_changes(self):
        expected = simulate_geometry()
        self.assert_samples_equal(simulate_geometry(simulation_scale=1), expected)
        node = cmds.ls(type="boneDynamicsNode")[0]
        target = cmds.listConnections(node + ".outputRotate", destination=True)[0]
        self.assertEqual(cmds.attributeQuery("simulationScale", node=node, listDefault=True), [1])
        # A same-frame scale change invalidates the output and clears old state.
        cmds.setAttr(node + ".enableTurbulence", False)
        cmds.setAttr(node + ".simulationScale", 0.01)
        self.assertEqual(cmds.getAttr(target + ".rotate")[0], (0, 0, 0))
        cmds.currentTime(13)
        actual = cmds.getAttr(target + ".rotate")[0]
        cmds.currentTime(0)
        cmds.getAttr(target + ".rotate")
        cmds.currentTime(1)
        self.assert_samples_equal([actual], cmds.getAttr(target + ".rotate"))

    def test_frame_duration_scales_gravity(self):
        # One step at different scene rates retains the actual elapsed duration.
        for unit, fps in (("film", 24), ("pal", 25), ("ntsc", 30), ("120fps", 120)):
            actual = simulate(1, 1, unit=unit)[-1]
            expected_angle = math.degrees(math.atan2(-980.0 / (fps * fps), 10.0))
            self.assertAlmostEqual(actual[2], expected_angle, delta=1e-7)

    def test_legacy_parameter_defaults_and_ranges(self):
        cmds.file(new=True, force=True)
        node, target = make_solver(1)
        self.assertEqual(cmds.getAttr(node + ".damping"), 0.1)
        for attr in ("damping", "stiffness"):
            self.assertEqual(cmds.attributeQuery(attr, node=node, minimum=True), [0])
            self.assertEqual(cmds.attributeQuery(attr, node=node, maximum=True), [1])

    def test_changing_frame_rate_resets_state(self):
        for mode in ("off", "serial", "parallel"):
            cmds.file(new=True, force=True)
            cmds.currentUnit(time="film")
            cmds.evaluationManager(mode=mode)
            node, target = make_solver(1)
            for frame in range(5):
                cmds.currentTime(frame)
                previous = cmds.getAttr(target + ".rotate")[0]
            self.assertGreater(abs(previous[2]), 0.001)
            cmds.currentUnit(time="pal", updateAnimation=False)
            # Ensure evaluation even when Maya preserves the current time value.
            cmds.setAttr(node + ".timeScale", 1.01)
            self.assertEqual(cmds.getAttr(target + ".rotate")[0], (0, 0, 0))
            cmds.setAttr(node + ".timeScale", 1)
            frame = cmds.currentTime(query=True)
            cmds.currentTime(frame + 1)
            actual = cmds.getAttr(target + ".rotate")[0]
            expected = simulate(1, 1, mode, "pal")[-1]
            for a, b in zip(actual, expected):
                self.assertAlmostEqual(a, b, delta=1e-7)

    def test_acceleration_integrates_the_full_duration(self):
        for mode in ("off", "serial", "parallel"):
            for scale in (2, 3.5, 10, 100):
                # Compare equal simulation durations with identical static inputs.
                accelerated = simulate(scale, 2, mode)[-1]
                steps = int(math.ceil(scale))
                reference = simulate(scale / steps, steps * 2, mode)[-1]
                self.assertGreater(max(abs(v) for v in accelerated), 0.001)
                for actual, expected in zip(accelerated, reference):
                    self.assertAlmostEqual(actual, expected, delta=1e-7,
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
            cmds.setAttr(node + ".damping", 1)
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
