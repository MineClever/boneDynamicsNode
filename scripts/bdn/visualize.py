from maya import cmds

from . import utils


def _find_visualizer(bone_dynamics_node, *args):
    # type: (str, *object) -> Tuple[Optional[str], Optional[str]]
    dest = cmds.listConnections(bone_dynamics_node, s=False, d=True, type='boneDynamicsVisualizer') or []
    if dest:
        node = dest[0]

        if cmds.nodeType(node) == 'boneDynamicsVisualizer':
            parents = cmds.listRelatives(node, p=True, f=False) or []
            if parents:
                return parents[0], node

        shapes = cmds.listRelatives(node, s=True, f=False) or []
        if shapes:
            if cmds.nodeType(shapes[0]) == 'boneDynamicsVisualizer':
                return node, shapes[0]

    return None, None


def _connect_attr(source, destination, *args):
    # type: (str, str, *object) -> None
    connected_sources = cmds.listConnections(destination, s=True, d=False, p=True) or []
    if source in connected_sources:
        return
    cmds.connectAttr(source, destination, f=True)


@utils.with_traceback
@utils.undo_chunk
def create_visualizer(bone_dynamics_node, **kwargs):
    # type: (str, **object) -> Tuple[str, str]
    """Create a viewport visualizer for a boneDynamicsNode.

    Args:
        bone_dynamics_node: Source boneDynamicsNode to visualize.
        **kwargs: Attribute values to set on the visualizer shape.

    Returns:
        A tuple containing the visualizer transform name and shape name.
    """

    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")

    visualizer_tm, visualizer_shape = _find_visualizer(bone_dynamics_node)

    if visualizer_shape is None:
        visualizer_tm, visualizer_shape = utils.create_shape_node('boneDynamicsVisualizer')

    utils.set_outliner_color(visualizer_tm, (0, .5, 1))
    utils.set_override_display_type(visualizer_shape, type='Reference')
    utils.set_attributes(visualizer_shape, **kwargs)

    cons = [
        ('outputRotate', 'outputRotate'),
        ('iterations', 'drawBoneCollision'),
        ('enableAngleLimit', 'drawAngleLimit'),
        ('enable', 'enable'),
        ('boneTranslate', 'boneTranslate'),
        ('boneJointOrient', 'boneJointOrient'),
        ('boneParentMatrix', 'boneParentMatrix'),
        ('boneParentInverseMatrix', 'boneParentInverseMatrix'),
        ('endTranslate', 'endTranslate'),
        ('rotationOffset', 'rotationOffset'),
        ('boneScale', 'boneScale'),
        ('boneInverseScale', 'boneInverseScale'),
        ('endScale', 'endScale'),
        ('radius', 'radius'),
        ('angleLimit', 'angleLimit'),
        ('rootRadius', 'rootRadius'),
        ('boneAsCapsule', 'boneAsCapsule')
    ]

    for src_attr, dst_attr in cons:
        _connect_attr('{}.{}'.format(bone_dynamics_node, src_attr), '{}.{}'.format(visualizer_shape, dst_attr))

    return visualizer_tm, visualizer_shape
