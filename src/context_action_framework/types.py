
from enum import IntEnum
from typing import List, Optional, Tuple
from dataclasses import dataclass
import numpy as np
from torch import Tensor
import torch
from shapely.geometry import Polygon

from geometry_msgs.msg import PoseStamped, Transform, Vector3, Quaternion

from context_action_framework.msg import Detection as ROSDetection
from context_action_framework.msg import Gap as ROSGap


class Action(IntEnum):
    none = 0
    cut = 1
    lever = 2
    move = 3
    push = 4
    turn_over = 5
    vision = 6


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

# todo: make Detection as close as possible to ROSDetection
@dataclass
class Detection:
    id: Optional[int] = None
    tracking_id: Optional[int] = None

    label: Optional[Label] = None
    score: Optional[float] = None

    tf_px: Optional[np.ndarray] = None
    box_px: Optional[np.ndarray] = None
    obb_px: Optional[np.ndarray] = None
    obb_3d_px: Optional[np.ndarray] = None

    tf: Optional[np.ndarray] = None
    box: Optional[np.ndarray] = None
    obb: Optional[np.ndarray] = None
    obb_3d: Optional[np.ndarray] = None

    polygon_px: Optional[Polygon] = None

    # stuff that we only use in vision internally
    mask: Optional[Tensor] = None
    mask_contour: Optional[np.ndarray] = None
    tracking_score: Optional[float] = None
    tracking_box: Optional[np.ndarray] = None


@dataclass
class LeverAction:
    id : Optional[int] = None
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



def detections_to_ros(detections):
    ros_detections = []
    for detection in detections:
        # todo: finish this
        
        polygon_exterior_coords = np.array(detection.polygon_px.exterior.coords)
        polygon_list = polygon_exterior_coords.ravel().tolist()
        
        # undo_ravel = np.asarray(polygon_list).reshape(-1, 2)
        # undo_ravel_success = np.array_equal(polygon_exterior_coords, undo_ravel)
        # print("undo_ravel_success", undo_ravel_success)
        
        ros_detection = ROSDetection(
            id = detection.id,
            tracking_id = detection.tracking_id,

            label = detection.label.value, 
            score = detection.score,
            
            tf_px = Transform(Vector3(*detection.tf_px[0], 0), Quaternion(*detection.tf_px[1])),
            box_px = detection.box_px.astype(float).ravel().tolist(),
            obb_px = detection.obb_px.astype(float).ravel().tolist(),
            # todo: obb_3d_px = ,

            tf = Transform(Vector3(*detection.tf_px[0], 0), Quaternion(*detection.tf_px[1])),
            box = detection.box.ravel().tolist(),
            obb = detection.obb.ravel().tolist(),
            # todo: obb_3d = ,

            polygon_px = polygon_list
        )
        ros_detections.append(ros_detection)
    
    return ros_detections

def detections_to_py(ros_detections):
    detections = []

    for ros_detection in ros_detections:
        detection = Detection(
            id = ros_detection.id,
            tracking_id = ros_detection.tracking_id,

            label = Label(ros_detection.label),
            score = ros_detection.score,

            tf_px = [ros_detection.tf_px.translation, ros_detection.tf_px.rotation],
            box_px = np.asarray(ros_detection.box_px).reshape(-1, 2),
            obb_px = np.asarray(ros_detection.obb_px).reshape(-1, 2),
            # todo: obb_3d_px = ,
            
            tf = [ros_detection.tf.translation, ros_detection.tf.rotation],
            box = np.asarray(ros_detection.box).reshape(-1, 2),
            obb = np.asarray(ros_detection.obb).reshape(-1, 2),
            # todo: obb_3d = ,

            polygon_px=Polygon(np.asarray(ros_detection.polygon_px).reshape(-1, 2)),
        )
        detections.append(detection)

    return detections

def gaps_to_ros(gaps):
    ros_gaps = []
    for gap in gaps:
        # todo: finish this
        ros_gap = ROSGap(
            id = gap.id,
            from_depth = gap.from_depth,
            to_depth = gap.to_depth
        )
    
        ros_gaps.append(ros_gap)
    
    return ros_gaps
