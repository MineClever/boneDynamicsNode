from maya import cmds

from . import utils, collider, visualize


@utils.with_traceback
@utils.undo_chunk
def create_dynamics_node(
        bone,
        end,
        scalable=False,
        target_bone=None,
        connect_target_scale=False,
        offset_node=None,
        create_visualizer=False,
        colliders=None,
        additional_force_node=None,
        additional_force_init_vec=(0, 0, -1),
        set_name='boneDynamicsNodeSet',
        **kwargs
    ):
    # type: (str, str, bool, Optional[str], Optional[bool], Optional[str], bool, Optional[Union[str, Sequence[str]]], Optional[str], Sequence[float], str, **object) -> str
    """Create and connect a boneDynamicsNode for a joint section.

    Args:
        bone: Joint driven by the created dynamics node.
        end: Tip joint that defines the end of the simulated section.
        scalable: Connect scale attributes when the joint should per-section scaling.
        target_bone: Optional node whose drives the rotation offset.
        connect_target_scale: Connect the scale from target_bone to bone.
            If the target pose is controlled using a duplicate joint, it is convenient
            to connect the scale from the duplicate joint as well.
        offset_node: Optional node whose offsets the simulation space.
        create_visualizer: Create and connect a visualizer.
        colliders: Collider transform name, or sequence of names.
        additional_force_node: Optional node whose world matrix controls additional force direction.
        additional_force_init_vec: Initial vector that controlled by additional_force_node.
        set_name: Object set that store the created dynamics node.
        **kwargs: Attribute values to set on the created boneDynamicsNode.
            For example, simulationScale=0.01 computes geometry at 1/100 size
            while keeping force parameter values in simulation space.

    Returns:
        The created boneDynamicsNode name.
    """

    # load plugin
    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")

    # validate input
    if cmds.nodeType(bone) != 'joint' or cmds.nodeType(end) != 'joint':
        raise ValueError("Exit: Both bone and end must be joints.")

    # check parent-child relationship
    bone_full_paths = cmds.ls(bone, l=True) or []
    bone_path = bone_full_paths[0] if bone_full_paths else bone
    end_parents = cmds.listRelatives(end, p=True, f=True)

    if not end_parents:
        raise ValueError("Exit: {} has no parent.".format(end))

    if bone_path not in end_parents:
        raise ValueError("Exit: {} is not {}'s parent.".format(bone, end))

    # create boneDynamicsNode and set attributes
    bone_dynamics_node = cmds.createNode("boneDynamicsNode")
    utils.set_attributes(bone_dynamics_node, **kwargs)

    # basic connections
    cmds.connectAttr('time1.outTime', '{}.time'.format(bone_dynamics_node), f=True)
    cmds.connectAttr('{}.translate'.format(bone), '{}.boneTranslate'.format(bone_dynamics_node), f=True)
    cmds.connectAttr('{}.parentMatrix[0]'.format(bone), '{}.boneParentMatrix'.format(bone_dynamics_node), f=True)
    cmds.connectAttr('{}.parentInverseMatrix[0]'.format(bone), '{}.boneParentInverseMatrix'.format(bone_dynamics_node), f=True)
    cmds.connectAttr('{}.jointOrient'.format(bone), '{}.boneJointOrient'.format(bone_dynamics_node), f=True)
    cmds.connectAttr('{}.translate'.format(end), '{}.endTranslate'.format(bone_dynamics_node), f=True)
    cmds.connectAttr('{}.outputRotate'.format(bone_dynamics_node), '{}.rotate'.format(bone), f=True)

    if scalable:
        cmds.connectAttr('{}.scale'.format(bone), '{}.boneScale'.format(bone_dynamics_node), f=True)
        cmds.connectAttr('{}.inverseScale'.format(bone), '{}.boneInverseScale'.format(bone_dynamics_node), f=True)
        cmds.connectAttr('{}.scale'.format(end), '{}.endScale'.format(bone_dynamics_node), f=True)

    # additional connections
    if target_bone and cmds.objExists(target_bone):
        cmds.connectAttr('{}.rotate'.format(target_bone), '{}.rotationOffset'.format(bone_dynamics_node), f=True)
        if connect_target_scale:
            cmds.connectAttr('{}.scale'.format(target_bone), '{}.scale'.format(bone), f=True)

    if offset_node and cmds.objExists(offset_node):
        cmds.connectAttr('{}.worldMatrix[0]'.format(offset_node), '{}.offsetMatrix'.format(bone_dynamics_node), f=True)

    if additional_force_node and cmds.objExists(additional_force_node):
        vp = cmds.listConnections('{}.worldMatrix[0]'.format(additional_force_node), s=False, d=True, type='vectorProduct')
        if vp:
            vp = vp[0]
        else:
            vp = cmds.createNode('vectorProduct')
            attrs = {
                "operation": 3,
                "input1": additional_force_init_vec,
                "normalizeOutput": 1,
            }
            utils.set_attributes(vp, **attrs)
            cmds.connectAttr('{}.worldMatrix[0]'.format(additional_force_node), '{}.matrix'.format(vp), f=True)
        cmds.connectAttr('{}.output'.format(vp), '{}.additionalForce'.format(bone_dynamics_node), f=True)

    # visualizer
    if create_visualizer:
        visualize.create_visualizer(bone_dynamics_node)

    # colliders
    if colliders is not None:
        if isinstance(colliders, utils.string_types):
            colliders = [colliders]
        else:
            colliders = list(colliders)
        collider.connect_colliders(bone_dynamics_node, colliders, replace=True)

    # add to object set
    if set_name:
        utils.add_object_to_set(bone_dynamics_node, set_name)

    return bone_dynamics_node
