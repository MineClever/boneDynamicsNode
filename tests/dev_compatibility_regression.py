"""mayapy dev_compatibility_regression.py plugin.mll [--capture]

Capture only with dev's plugin; ordinary runs compare against the stored dev
outputs. Each case includes moving parent/offset inputs and a nonzero start frame.
The checked-in baseline was captured with Maya 2022 using dev d5179a7's
committed plug-ins/2022/boneDynamicsNode.mll.
"""
import json
import math
import os
import sys

import maya.standalone
maya.standalone.initialize(name="python")
from maya import cmds


def capture(with_mesh=False, reverse=False):
    result = []
    for elastic in (0, 1, 2):
        for damping, stiffness in ((0.1, 0), (0.5, 0.1), (1, 1)):
            for turbulence in (False, True):
                cmds.file(new=True, force=True)
                cmds.currentUnit(time="ntsc")
                cmds.evaluationManager(mode="off")
                node = cmds.createNode("boneDynamicsNode")
                cmds.connectAttr("time1.outTime", node + ".time")
                for attr, value in (("damping", damping), ("stiffness", stiffness),
                                    ("elasticity", 399), ("elasticForceFunction", elastic),
                                    ("gravityMultiply", 1), ("enableTurbulence", turbulence),
                                    ("turbulenceSeed", 7), ("turbulenceStrength", 50)):
                    cmds.setAttr(node + "." + attr, value)
                cmds.setAttr(node + ".endTranslate", 10, 0, 0)
                cmds.setAttr(node + ".additionalForce", 0, 0, 12)
                parent = cmds.createNode("transform")
                offset = cmds.createNode("transform")
                cmds.connectAttr(parent + ".worldMatrix[0]", node + ".boneParentMatrix")
                cmds.connectAttr(parent + ".worldInverseMatrix[0]", node + ".boneParentInverseMatrix")
                cmds.connectAttr(offset + ".worldMatrix[0]", node + ".offsetMatrix")
                cmds.setAttr(node + ".offsetMatrixWeight", 0.65)
                if with_mesh:
                    mesh = cmds.polyPlane(width=100, height=100)[0]
                    cmds.setAttr(mesh + ".translateY", -10000)
                    shape = cmds.listRelatives(mesh, shapes=True)[0]
                    provider = cmds.createNode("meshColliderNode")
                    cmds.connectAttr(shape + ".worldMesh[0]", provider + ".inputMesh")
                    cmds.connectAttr("time1.outTime", provider + ".time")
                    cmds.connectAttr(provider + ".outputCollider", node + ".meshColliderData[0]")
                rows = []
                frames = range(112, 99, -1) if reverse else range(100, 113)
                for frame in frames:
                    cmds.setAttr(parent + ".rotateY", (frame - 100) * 2)
                    cmds.setAttr(offset + ".translateY", (frame - 100) * 0.2)
                    cmds.currentTime(frame)
                    rows.append(cmds.getAttr(node + ".outputRotate")[0])
                result.append(rows)
    return result


if __name__ == "__main__":
    try:
        cmds.loadPlugin(os.path.abspath(sys.argv[1]))
        reverse = "--reverse" in sys.argv
        actual = capture(reverse=reverse)
        fixture = os.path.join(os.path.dirname(__file__),
                               "dev_solver_reverse_baseline.json" if reverse else "dev_solver_baseline.json")
        if "--capture" in sys.argv:
            with open(fixture, "w") as stream:
                json.dump(actual, stream, indent=2)
            print("Captured 18 cases from dev plugin")
        else:
            with open(fixture) as stream:
                expected = json.load(stream)
            assert len(actual) == len(expected)
            error = max(abs(a - b) for aa, bb in zip(actual, expected)
                        for row_a, row_b in zip(aa, bb) for a, b in zip(row_a, row_b))
            assert not math.isnan(error) and not math.isinf(error) and error < 1e-7, error
            distant = capture(with_mesh=True, reverse=reverse)
            mesh_error = max(abs(a - b) for aa, bb in zip(distant, expected)
                             for row_a, row_b in zip(aa, bb) for a, b in zip(row_a, row_b))
            assert not math.isnan(mesh_error) and not math.isinf(mesh_error) and mesh_error < 1e-7, mesh_error
            print("18 distant-mesh cases passed; max difference = {} degrees".format(mesh_error))
            print("18 dev compatibility cases passed; max difference = {} degrees".format(error))
    finally:
        maya.standalone.uninitialize()
