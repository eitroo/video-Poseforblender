bl_info = {
    "name": "Video to Pose Map",
    "author": "Hoang Nguyen The & Oleg Kozevnikov",
    "version": (1, 0, 0),
    "blender": (2, 90, 0),
    "location": "Object Properties > 3D Pose Map",
    "description": "Video based pose estimation mapped to armature",
    "category": "Animation"}

from . import panels
from . import operators
from . import props


def register():
    panels.register()
    operators.register()
    props.register()


def unregister():
    panels.unregister()
    operators.unregister()
    props.unregister()


if __name__ == "__main__":
    register()
