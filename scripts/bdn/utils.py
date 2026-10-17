import traceback
import functools
import re

from maya import cmds

try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


# decorator
def undo_chunk(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        cmds.undoInfo(openChunk=True)
        try:
            result = func(*args, **kwargs)
        finally:
            cmds.undoInfo(closeChunk=True)
        return result
    return wrapper


def with_traceback(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            print("Error occurred in '{}'".format(func.__name__))
            traceback.print_exc()
            raise
    return wrapper


# utils
def load_plugin(*args):
    # type: (*object) -> bool
    try:
        # An explicitly loaded absolute path need not be on MAYA_PLUG_IN_PATH.
        if 'boneDynamicsNode' in (cmds.pluginInfo(query=True, listPlugins=True) or []):
            return True
        result = cmds.loadPlugin("boneDynamicsNode.mll", qt=True)
        if result is not None:
            plugin_ver = cmds.pluginInfo("boneDynamicsNode.mll", q=True, version=True)
            print("Plugin loaded successfully. Version: {}".format(plugin_ver))
        return True
    except Exception as e:
        print("Error loading plugin: {}".format(e))
        return False


def get_fps(*args):
    # type: (*object) -> float
    time_unit = cmds.currentUnit(q=True, time=True)
    fps_match = re.match(r'^([0-9]+(?:\.[0-9]+)?)fps$', time_unit)
    if fps_match:
        fps = float(fps_match.group(1))
        if fps > 0:
            return fps

    fps_mapping = {
        'game': 15.0,
        'film': 24.0,
        'pal': 25.0,
        'ntsc': 30.0,
        'show': 48.0,
        'palf': 50.0,
        'ntscf': 60.0
    }
    return fps_mapping.get(time_unit, 30.0)


def create_shape_node(node_name, *args):
    # type: (str, *object) -> Tuple[str, str]

    def _unique_name(name):
        i = 1
        while True:
            if cmds.objExists('{}{}'.format(name, i)):
                i += 1
            else:
                break
        return '{}{}'.format(name, i)

    transform_unique_name = _unique_name(node_name)

    shape = cmds.createNode(node_name, name=_unique_name('{}Shape'.format(node_name)))
    transform_fullpath = cmds.listRelatives(shape, p=True, f=True)[0]
    transform = transform_fullpath.split('|')[-1]

    if transform != transform_unique_name:
        transform = cmds.rename(transform_fullpath, transform_unique_name)
        shape = cmds.listRelatives(transform, s=True, f=True)[0]

    shape = cmds.rename(shape, transform + "Shape")

    return transform, shape


def set_attributes(node, **attributes):
    # type: (str, **Any) -> None
    for at, v in attributes.items():
        if not cmds.attributeQuery(at, n=node, ex=True):
            print("Attribute does not exist: {}.{}".format(node, at))
            continue
        try:
            if isinstance(v, string_types):
                cmds.setAttr("{}.{}".format(node, at), v, type="string")

            elif (type(v) == list or type(v) == tuple) and len(v) >= 3:
                cmds.setAttr("{}.{}".format(node, at), v[0], v[1], v[2], type="double3")

            else:
                cmds.setAttr("{}.{}".format(node, at), v)
        except:
            print("Failed to set attribute: {}.{}".format(node, at))


def set_outliner_color(node, color, *args):
    # type: (str, Sequence[float], *object) -> None
    cmds.setAttr('{}.useOutlinerColor'.format(node), True)
    cmds.setAttr('{}.outlinerColor'.format(node), color[0], color[1], color[2])


def set_override_display_color(node, color_idx, *args):
    # type: (str, int, *object) -> None
    cmds.setAttr('{}.overrideEnabled'.format(node), True)
    cmds.setAttr('{}.overrideRGBColors'.format(node), False)
    cmds.setAttr('{}.overrideColor'.format(node), color_idx)


def set_override_display_type(node, type, *args):
    # type: (str, str, *object) -> None
    if type == 'Reference':
        display_type = 2
    elif type == 'Template':
        display_type = 1
    else:
        display_type = 0

    set_override_display_color(node, 0)
    cmds.setAttr('{}.overrideDisplayType'.format(node), display_type)


def add_object_to_set(obj, set_name, *args):
    # type: (str, str, *object) -> None

    if not set_name:
        return

    if not cmds.objExists(set_name):
        cmds.select(cl=True)
        cmds.sets(name=set_name)

    cmds.sets(obj, add=set_name)
