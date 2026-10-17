"""Run in Maya after loading the rebuilt plugin.

Play every frame over frames 1-48, then scrub both ways. The sphere radius,
capsule end radii and capsule height should follow their keys without selecting
the shapes or editing attributes. Repeat in DG, Serial and Parallel modes.
Creates a separate group and leaves existing scene objects in place.
"""
from maya import cmds


group = cmds.group(empty=True, name="animatedColliderPreview#")
for kind, x, animations in (
    ("sphereColliderNode", -5, {"radius": (1, 3, 1)}),
    ("capsuleColliderNode", 5, {"radiusA": (1, 3, 1),
                                "radiusB": (2, 0.5, 2),
                                "height": (3, 7, 3)}),
):
    shape = cmds.createNode(kind)
    transform = cmds.listRelatives(shape, parent=True, fullPath=True)[0]
    transform = cmds.parent(transform, group)[0]
    cmds.setAttr(transform + ".translateX", x)
    for attr, values in animations.items():
        for frame, value in zip((1, 24, 48), values):
            cmds.setKeyframe(shape, attribute=attr, time=frame, value=value)
        cmds.keyTangent(shape, attribute=attr, inTangentType="linear",
                        outTangentType="linear")

cmds.playbackOptions(minTime=1, maxTime=48, playbackSpeed=0, by=1)
cmds.currentTime(1)
cmds.select(clear=True)
print("Collider preview created: " + group)
