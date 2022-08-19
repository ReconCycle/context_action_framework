from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple
import numpy as np
from torch import Tensor
from geometry_msgs.msg import PoseStamped


class Action(IntEnum):
    move = 0
    cut = 1
    lever = 2
    turn_over = 3
    remove_clip = 4


@dataclass
class Detection:
    id: Optional[int] = None
    label: Optional[IntEnum] = None
    
    score: Optional[float] = None
    box: Optional[np.ndarray] = None
    mask: Optional[Tensor] = None
    mask_contour: Optional[np.ndarray] = None
    mask_polygon: Optional[np.ndarray] = None
    
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