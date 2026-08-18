from maya import cmds

from . import utils


@utils.with_traceback
@utils.undo_chunk
def create_sphere_collider(radius=1.0, *args, **kwargs):
    # type: (float, *object, **object) -> Tuple[str, str]
    """Create a sphere collider.

    Args:
        radius: Initial value for the collider shape's radius attribute.

    Returns:
        A tuple containing the transform name and shape name.
    """

    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")

    transform, shape = utils.create_shape_node('sphereColliderNode')
    utils.set_outliner_color(transform, (1, 1, 0))
    utils.set_override_display_color(shape, 17)
    cmds.setAttr("{}.radius".format(shape), radius)

    return transform, shape


@utils.with_traceback
@utils.undo_chunk
def create_capsule_collider(height=1.0, radius_a=1.0, radius_b=1.0, *args, **kwargs):
    # type: (float, float, float, *object, **object) -> Tuple[str, str]
    """Create a capsule collider.

    Args:
        height: Initial value for the collider shape's height attribute.
        radius_a: Initial radius at the first capsule end.
        radius_b: Initial radius at the second capsule end.

    Returns:
        A tuple containing the transform name and shape name.
    """

    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")

    transform, shape = utils.create_shape_node('capsuleColliderNode')
    utils.set_outliner_color(transform, (1, 1, 0))
    utils.set_override_display_color(shape, 17)
    cmds.setAttr("{}.height".format(shape), height)
    cmds.setAttr("{}.radiusA".format(shape), radius_a)
    cmds.setAttr("{}.radiusB".format(shape), radius_b)

    return transform, shape


@utils.with_traceback
@utils.undo_chunk
def create_infinite_plane_collider(*args, **kwargs):
    # type: (*object, **object) -> Tuple[str, str]
    """Create an infinite plane collider.

    Returns:
        A tuple containing the transform name and shape name.
    """

    if not utils.load_plugin():
        raise RuntimeError("Failed to load boneDynamicsNode plugin.")

    transform, shape = utils.create_shape_node('infinitePlaneColliderNode')
    utils.set_outliner_color(transform, (1, 1, 0))
    utils.set_override_display_color(shape, 17)

    return transform, shape


@utils.with_traceback
@utils.undo_chunk
def disconnect_colliders(bone_dynamics_node, collider_attributes=None, *args):
    # type: (str, Optional[List[str]], *object) -> None
    """Remove collider connections from a boneDynamicsNode.

    Args:
        bone_dynamics_node: boneDynamicsNode to disconnect from colliders.
        collider_attributes: Multi attributes to clear.
            When omitted, all supported collider attributes are cleared.
    """

    if collider_attributes is None:
        collider_attributes = [
            'sphereCollider',
            'capsuleCollider',
            'capsuleColliderInput',
            'infinitePlaneCollider',
            'meshCollider'
        ]

    for col_attr in collider_attributes:
        indices = cmds.getAttr('{}.{}'.format(bone_dynamics_node, col_attr), mi=True)
        if indices is None:
            continue
        for idx in indices:
            cmds.removeMultiInstance('{}.{}[{}]'.format(bone_dynamics_node, col_attr, idx), b=True)


class _ColliderConnector:

    def __init__(self, bone_dynamics_node):
        # type: (str) -> None
        self.bone_dynamics_node = bone_dynamics_node  # type: str
        self._update_indices()

    def _update_indices(self):
        # type: () -> None

        def _get_next_index(attr):
            # type: (str) -> int
            indices = cmds.getAttr('{}.{}'.format(self.bone_dynamics_node, attr), mi=True)
            return max(indices) + 1 if indices else 0

        self.sphere_col_idx = _get_next_index('sphereCollider')  # type: int
        self.capsule_col_idx = _get_next_index('capsuleCollider')  # type: int
        self.capsule_col_input_idx = _get_next_index('capsuleColliderInput')  # type: int
        self.iplane_col_idx = _get_next_index('infinitePlaneCollider')  # type: int
        self.mesh_col_idx = _get_next_index('meshCollider')  # type: int

    def connect_sphere(self, matrix_attr, radius_attr):
        # type: (str, str) -> None
        cmds.connectAttr(matrix_attr, '{}.sphereCollider[{}].sphereColMatrix'.format(self.bone_dynamics_node, self.sphere_col_idx), f=True)
        cmds.connectAttr(radius_attr, '{}.sphereCollider[{}].sphereColRadius'.format(self.bone_dynamics_node, self.sphere_col_idx), f=True)
        self.sphere_col_idx += 1

    def connect_capsule_input(self, matrix_attr, height_attr, radius_a_attr, radius_b_attr):
        # type: (str, str, str, str) -> None
        cmds.connectAttr(matrix_attr, '{}.capsuleColliderInput[{}].capsuleColliderMatrix'.format(self.bone_dynamics_node, self.capsule_col_input_idx), f=True)
        cmds.connectAttr(height_attr, '{}.capsuleColliderInput[{}].capsuleColliderHeight'.format(self.bone_dynamics_node, self.capsule_col_input_idx), f=True)
        cmds.connectAttr(radius_a_attr, '{}.capsuleColliderInput[{}].capsuleColliderRadiusA'.format(self.bone_dynamics_node, self.capsule_col_input_idx), f=True)
        cmds.connectAttr(radius_b_attr, '{}.capsuleColliderInput[{}].capsuleColliderRadiusB'.format(self.bone_dynamics_node, self.capsule_col_input_idx), f=True)
        self.capsule_col_input_idx += 1

    def connect_capsule(self, matrix_a_attr, matrix_b_attr, radius_a_attr, radius_b_attr):
        # type: (str, str, str, str) -> None
        cmds.connectAttr(matrix_a_attr, '{}.capsuleCollider[{}].capsuleColMatrixA'.format(self.bone_dynamics_node, self.capsule_col_idx), f=True)
        cmds.connectAttr(matrix_b_attr, '{}.capsuleCollider[{}].capsuleColMatrixB'.format(self.bone_dynamics_node, self.capsule_col_idx), f=True)
        cmds.connectAttr(radius_a_attr, '{}.capsuleCollider[{}].capsuleColRadiusA'.format(self.bone_dynamics_node, self.capsule_col_idx), f=True)
        cmds.connectAttr(radius_b_attr, '{}.capsuleCollider[{}].capsuleColRadiusB'.format(self.bone_dynamics_node, self.capsule_col_idx), f=True)
        self.capsule_col_idx += 1

    def connect_infinite_plane(self, matrix_attr):
        # type: (str) -> None
        cmds.connectAttr(matrix_attr, '{}.infinitePlaneCollider[{}].infinitePlaneColMatrix'.format(self.bone_dynamics_node, self.iplane_col_idx), f=True)
        self.iplane_col_idx += 1

    def connect_mesh(self, mesh_attr):
        # type: (str) -> None
        cmds.connectAttr(mesh_attr, '{}.meshCollider[{}]'.format(self.bone_dynamics_node, self.mesh_col_idx), f=True)
        self.mesh_col_idx += 1


@utils.with_traceback
@utils.undo_chunk
def connect_colliders(bone_dynamics_node, colliders, replace=False, *args):
    # type: (str, Union[str, Sequence[str]], bool, *object) -> None
    """Connect supported collider nodes to a boneDynamicsNode.

    Args:
        bone_dynamics_node: boneDynamicsNode that receives collider inputs.
        colliders: Collider transform name, mesh transform name, or sequence of names.
        replace: Disconnect existing collider inputs before connecting the given colliders.

    Notes:
        Supported inputs include bdn sphere, capsule, and infinite-plane collider transforms,
            mesh transforms, and expcol collider nodes.
    """

    if not colliders:
        return

    if isinstance(colliders, utils.string_types):
        colliders = [colliders]
    else:
        colliders = list(colliders)

    if replace:
        disconnect_colliders(bone_dynamics_node)

    connector = _ColliderConnector(bone_dynamics_node)

    # collider loop
    for col in colliders:

        if not cmds.objExists(col):
            print("Skip: {} is not found.".format(col))
            continue

        is_found = False

        col_shape = cmds.listRelatives(col, s=True, f=True)

        if col_shape:
            col_shape = col_shape[0]
            col_node_type = cmds.nodeType(col_shape)

            if col_node_type == 'sphereColliderNode':
                connector.connect_sphere('{}.worldMatrix[0]'.format(col), '{}.radius'.format(col_shape))
                is_found = True

            elif col_node_type == 'capsuleColliderNode':
                connector.connect_capsule_input('{}.worldMatrix[0]'.format(col), '{}.height'.format(col_shape), '{}.radiusA'.format(col_shape), '{}.radiusB'.format(col_shape))
                is_found = True

            elif col_node_type == 'infinitePlaneColliderNode':
                connector.connect_infinite_plane('{}.worldMatrix[0]'.format(col))
                is_found = True

            elif col_node_type == 'mesh':
                connector.connect_mesh('{}.worldMesh[0]'.format(col_shape))
                is_found = True

        if is_found:
            continue

        # expcol-type colliders
        if cmds.attributeQuery('colliderType', n=col, ex=True):
            # get expCol's collider type
            collider_type = cmds.getAttr('{}.colliderType'.format(col))

            if collider_type == 'sphere':
                connector.connect_sphere('{}.worldMatrix[0]'.format(col), '{}.radius'.format(col))
                is_found = True

            elif collider_type in ['capsule', 'capsule2']:
                ra = ".radius" if collider_type == 'capsule' else ".radiusA"
                rb = ".radius" if collider_type == 'capsule' else ".radiusB"
                a = cmds.listConnections('{}.sphereA'.format(col), s=True, d=False) or []
                b = cmds.listConnections('{}.sphereB'.format(col), s=True, d=False) or []
                if not a or not b:
                    print("Skip: {} capsule collider is missing sphereA or sphereB connection.".format(col))
                    continue
                a = a[0]
                b = b[0]
                connector.connect_capsule('{}.worldMatrix[0]'.format(a), '{}.worldMatrix[0]'.format(b), '{}{}'.format(col, ra), '{}{}'.format(col, rb))
                is_found = True

            elif collider_type == 'infinitePlane':
                connector.connect_infinite_plane('{}.worldMatrix[0]'.format(col))
                is_found = True

        if not is_found:
            print("Skip: {} is not a supported collider type.".format(col))
