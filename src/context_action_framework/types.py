
from email.policy import default

from enum import IntEnum
from typing import List, Optional, Tuple
import numpy as np
from torch import Tensor
import torch
from geometry_msgs.msg import PoseStamped, Transform
from shapely.geometry import Polygon

from dataclasses import dataclass, field
# from dataclasses_json import dataclass_json, config, Undefined
# from pydantic.dataclasses import dataclass

class Action(IntEnum):
    cut = 0
    lever = 1
    move = 2
    push = 3
    turn_over = 4
    vision = 5


class Label(IntEnum):
    hca_front = 0
    hca_back = 1
    hca_side1 = 2
    hca_side2 = 3
    battery = 4
    pcb = 5
    internals = 6
    pcb_covered = 7
    plastic_clip = 8


class Robot(IntEnum):
    panda1 = 1
    panda2 = 2


class Module(IntEnum):
    vision = 0
    panda1 = 1
    panda2 = 2
    vice = 3
    cutter = 4


class EndEffector(IntEnum):
    soft_hand = 0
    soft_gripper = 1
    screwdriver = 2


class Camera(IntEnum):
    basler = 0
    realsense = 1


@dataclass
class Detection:
    id: Optional[int] = None
    label: Optional[Label] = None
    
    score: Optional[float] = None
    box: Optional[np.ndarray] = None
    mask: Optional[Tensor] = None
    mask_contour: Optional[np.ndarray] = None
    mask_polygon: Optional[Polygon] = None

    obb_corners: Optional[np.ndarray] = None
    obb_center: Optional[np.ndarray] = None
    obb_rot_quat: Optional[np.ndarray] = None
    obb_corners_meters: Optional[np.ndarray] = None
    obb_center_meters: Optional[np.ndarray] = None
    
    tracking_id: Optional[int] = None
    tracking_score: Optional[float] = None
    tracking_box: Optional[np.ndarray] = None


@dataclass
class LeverAction:
    from_px: Optional[np.ndarray] = None
    to_px: Optional[np.ndarray] = None

    from_depth: Optional[float] = None
    to_depth: Optional[float] = None

    # in camera coords
    from_camera: Optional[np.ndarray] = None
    to_camera: Optional[np.ndarray] = None

    obb_px: Optional[np.ndarray] = None
    bb_camera: Optional[np.ndarray] = None

    pose_stamped: Optional[PoseStamped] = None


# @dataclass
# class MoveBlock:
#     from_module: Optional[Module] = None
#     from_tf: Optional[Transform] = None
#     to_module: Optional[Module] = None
#     to_tf: Optional[Transform] = None
#     object_obb: Optional[np.ndarray] = None
#     robot: Optional[Robot] = None
#     end_effector: Optional[EndEffector] = None


# @dataclass
# class LeverBlock:
#     module: Optional[Module] = None
#     from_tf: Optional[Transform] = None
#     to_tf: Optional[Transform] = None
#     object_obb: Optional[np.ndarray] = None
#     robot: Optional[Robot] = None
#     end_effector: Optional[EndEffector] = None


# @dataclass
# class CutBlock:
#     from_module: Optional[Module] = None
#     from_tf: Optional[Transform] = None
#     to_module: Optional[Module] = None
#     to_tf: Optional[Transform] = None
#     object_obb: Optional[np.ndarray] = None
#     robot: Optional[Robot] = None
#     end_effector: Optional[EndEffector] = None
#     # todo: specify where to make the cut, including which side to insert into cutter


# @dataclass
# class TurnOverBlock:
#     module: Optional[Module] = None
#     tf: Optional[Transform] = None
#     object_obb: Optional[np.ndarray] = None
#     robot: Optional[Robot] = None
#     end_effector: Optional[EndEffector] = None


# @dataclass
# class PushBlock:
#     module: Optional[Module] = None
#     from_tf: Optional[Transform] = None
#     to_tf: Optional[Transform] = None
#     robot: Optional[Robot] = None
#     end_effector: Optional[EndEffector] = None


# @dataclass
# class VisionBlock:
#     """Vision Block
    
#     Constructor arguments:
#     :param tf: tf of camera
#     """
#     camera: Optional[Camera] = None
#     module: Optional[Module] = None
#     tf: Optional[Transform] = None
#     gap_detection: Optional[bool] = None
