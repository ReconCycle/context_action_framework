
from enum import IntEnum
from typing import List, Optional, Tuple, Any
from dataclasses import dataclass
import numpy as np
from shapely.geometry import Polygon
import json

from rospy_message_converter import message_converter
from geometry_msgs.msg import PoseStamped, Transform, Vector3, Quaternion

from context_action_framework.msg import Detection as ROSDetection
from context_action_framework.msg import Gap as ROSGap


class Action(IntEnum):
    none = 0
    start = 1
    end = 2
    
    cut = 3
    lever = 4
    move = 5
    push = 6
    turn_over = 7
    vision = 8
    vice = 9


class Label(IntEnum):
    hca = 0
    smoke_detector = 1
    battery = 2
    pcb = 3
    internals = 4
    pcb_covered = 5
    plastic_clip = 6
    wires = 7
    screw = 8
    battery_covered = 9
    gap = 10
    
    hca_empty = 11
    smoke_detector_insides = 12
    smoke_detector_insides_empty = 13

class LabelFace(IntEnum):
    front = 0
    back = 1
    side1 = 2
    side2 = 3

class Robot(IntEnum):
    panda1 = 1
    panda2 = 2


class Module(IntEnum):
    vision = 0
    panda_1 = 1
    panda_2 = 2
    vise = 3
    cutter = 4
    cnc = 5


class EndEffector(IntEnum):
    soft_hand = 0
    soft_gripper = 1
    screwdriver = 2


class Camera(IntEnum):
    basler = 0
    realsense = 1

@dataclass
class Location:
    module: Optional[Module] = None
    tf: Optional[Transform] = None

# shorthand function
def _trans(vect, quat):
    return Transform(Vector3(*vect), Quaternion(*quat))

@dataclass
class Locations:
    # realsense locations for looking at vice and at the whole vice module
    vision_above_vice = Location(Module.vise, _trans([0.3, 0.3, 0.4], [0.0, 0.0, -1.0, 0.0]))
    vision_above_vice_module = Location(Module.vise, _trans([0.3, 0.3, 0.6], [0.0, 0.0, -1.0, 0.0]))

    # for pushing out the pin
    push_device_pin_start = Location(Module.vise, _trans([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])) # todo
    push_device_pin_end = Location(Module.vise, _trans([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])) # todo

    # tf of where softhand can drop object into vice slide. Needed for moveblock
    move_vice_slide = Location(Module.vise, _trans([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])) # todo

    # entrance of the cutter
    cutter_entrance = Location(Module.cutter, _trans([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])) # todo

    # finish locations
    battery_finish = Location(Module.cutter, _trans([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])) # todo
    plastic_finish = Location(Module.cutter, _trans([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])) # todo
    pcb_finish = Location(Module.cutter, _trans([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])) # todo



@dataclass
class Detection:
    id: Optional[int] = None                   # index in detections list
    tracking_id: Optional[int] = None          # unique ID per label that is stable across frames.

    label: Optional[Label] = None              # hca/smoke_detector/battery/internals/...
    label_face: Optional[LabelFace] = None     # front/back/side1/side2
    label_precise: Optional[str] = None        # 01/01.1/03.1/...
    label_precise_name: Optional[str] = None   # kalo/minal/fumonic/siemens/...
    score: Optional[float] = None

    tf_px: Optional[Transform] = None
    box_px: Optional[np.ndarray] = None
    obb_px: Optional[np.ndarray] = None
    center_px: Optional[np.ndarray] = None
    polygon_px: Optional[Polygon] = None

    edge_px_small: Optional[float] = None
    edge_px_large: Optional[float] = None

    tf: Optional[Transform] = None
    box: Optional[np.ndarray] = None
    obb: Optional[np.ndarray] = None
    center: Optional[np.ndarray] = None
    polygon: Optional[Polygon] = None

    edge_small: Optional[float] = None
    edge_large: Optional[float] = None

    obb_3d: Optional[np.ndarray] = None

    # stuff that we only use in vision internally
    mask: Optional[Any] = None
    mask_contour: Optional[np.ndarray] = None
    tracking_score: Optional[float] = None
    tracking_box: Optional[np.ndarray] = None

    parent_frame: Optional[str] = None
    table_name: Optional[str] = None
    tf_name: Optional[str] = None

    valid: Optional[str] = None
    is_invalid_reason: Optional[str] = None


@dataclass
class Gap:
    id : Optional[int] = None

    from_tf: Optional[Transform] = None
    to_tf: Optional[Transform] = None

    from_depth: Optional[float] = None
    to_depth: Optional[float] = None

    from_det: Optional[Detection] = None
    to_det: Optional[Detection] = None
    
    obb: Optional[np.ndarray] = None
    obb_3d: Optional[np.ndarray] = None
    
    # internal stuff
    from_px: Optional[np.ndarray] = None
    to_px: Optional[np.ndarray] = None

    # # in camera coords
    # from_camera: Optional[np.ndarray] = None
    # to_camera: Optional[np.ndarray] = None

    # obb_px: Optional[np.ndarray] = None
    # bb_camera: Optional[np.ndarray] = None

    # pose_stamped: Optional[PoseStamped] = None

classify_mapping_smoke_dets = {
    "01": "senys",
    "02": "fumonic",
    "03": "siemens",
    "04": "hekatron",
    "05": "kalo",
    "06": "fireangel",
    "07": "siemens2",
    "08": "zettler",
    "09": "honeywell",
    "10": "esser",
}
classify_mapping_hcas = {
    "01": "kalo2",
    "02": "minol",
    "03": "kalo",
    "04": "techem",
    "05": "ecotron",
    "06": "heimer",
    "07": "caloric",
    "08": "exim",
    "09": "ista",
    "10": "qundis",
    "11": "enco",
    "12": "kundo",
    "13": "qundis2",
}

def lookup_label_precise_name(label, label_precise):

    classify_num_before_dot = label_precise.rsplit(".")[0]

    label_precise_name = None
    if label == Label.hca:
        if classify_num_before_dot in classify_mapping_hcas:
            label_precise_name = classify_mapping_hcas[classify_num_before_dot]

    elif label == Label.smoke_detector:
        if classify_num_before_dot in classify_mapping_hcas:
            label_precise_name = classify_mapping_smoke_dets[classify_num_before_dot]

    return label_precise_name


def classify_name_to_class_id(classify_name):
    """
    convert smokedet_kalo to 05
    """
    name_split = classify_name.rsplit('_')
    name_type = name_split[0]
    name = name_split[1]

    if name_type in ["firealarm", "smokedet"]:
        # which number does `allow_item_name' it belong to?
        name_type = "firealarm" # in this codebase it is always called firealarm, never smokedet
        class_id =  list(classify_mapping_smoke_dets.keys())[list(classify_mapping_smoke_dets.values()).index(name)]
    elif name_type == "hca":
        class_id = list(classify_mapping_hcas.keys())[list(classify_mapping_hcas.values()).index(name)]

    return name_type, class_id


def detections_to_ros(detections):
    ros_detections = []
    for detection in detections:
        # undo_ravel = np.asarray(polygon_list).reshape(-1, 2)
        # undo_ravel_success = np.array_equal(polygon_exterior_coords, undo_ravel)
        # print("undo_ravel_success", undo_ravel_success)
        polygon_px = []
        if detection.polygon_px is not None:
            polygon_px = np.array(detection.polygon_px.exterior.coords).ravel().tolist()
        
            if len(polygon_px) % 2 == 1:
                print("SOMETHING WENT WRONG WITH POLYGON RAVEL. Not divisible by 2.")
                print("len(ros_detection.polygon_px)", len(polygon_px))
                print("shape detection.polygon_px.exterior.coords", np.array(detection.polygon_px.exterior.coords).shape)
        
        polygon = []
        if detection.polygon is not None:
            polygon = np.array(detection.polygon.exterior.coords).ravel().tolist()
            if len(polygon) % 3 == 1:
                print("SOMETHING WENT WRONG WITH POLYGON RAVEL. Not divisible by 3.")
                print("len(ros_detection.polygon)", len(polygon))
                print("shape detection.polygon.exterior.coords", np.array(detection.polygon.exterior.coords).shape)

        # print("detection.box.shape", detection.box.shape)
        # print("detection.obb.shape", detection.obb.shape)
        # print("detection.center.shape", detection.center.shape)
        # print("detection.obb_3d.shape", detection.obb_3d.shape)

        ros_detection = ROSDetection(
            id = detection.id,
            tracking_id = detection.tracking_id,

            label = detection.label.value,
            label_face = detection.label_face.value if detection.label_face is not None else None,
            label_precise = detection.label_precise,
            label_precise_name = detection.label_precise_name,
            score = detection.score,
            
            tf_px = detection.tf_px,
            # tf_px = Transform(Vector3(*detection.tf_px[0], 0), Quaternion(*detection.tf_px[1])),
            box_px = detection.box_px.astype(float).ravel().tolist() if detection.box_px is not None else [],
            obb_px = detection.obb_px.astype(float).ravel().tolist() if detection.obb_px is not None else [],
            center_px = detection.center_px.astype(float).tolist() if detection.center_px is not None else [],
            polygon_px = polygon_px,

            tf = detection.tf,
            # tf = Transform(Vector3(*detection.tf_px[0], 0), Quaternion(*detection.tf_px[1])),
            box = detection.box.ravel().tolist() if detection.box is not None else [],
            obb = detection.obb.ravel().tolist() if detection.obb is not None else [],
            center = detection.center.tolist() if detection.center is not None else [],
            polygon = polygon,
            
            obb_3d = detection.obb_3d.ravel().tolist() if detection.obb_3d is not None else [],

            parent_frame = detection.parent_frame,
            table_name = detection.table_name,
            tf_name = detection.tf_name,
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
            label_face = LabelFace(ros_detection.label_face) if ros_detection.label_face is not None else None,
            label_precise = ros_detection.label_precise,
            label_precise_name = ros_detection.label_precise_name,
            score = ros_detection.score,

            tf_px = ros_detection.tf_px,
            # tf_px = [ros_detection.tf_px.translation, ros_detection.tf_px.rotation],
            box_px = np.asarray(ros_detection.box_px).reshape(-1, 2),
            obb_px = np.asarray(ros_detection.obb_px).reshape(-1, 2),
            center_px = np.asarray(ros_detection.center_px),
            polygon_px = Polygon(np.asarray(ros_detection.polygon_px).reshape(-1, 2)),
            
            tf = ros_detection.tf,
            # tf = [ros_detection.tf.translation, ros_detection.tf.rotation],
            
            box = np.asarray(ros_detection.box).reshape(-1, 2),
            obb = np.asarray(ros_detection.obb).reshape(-1, 2),
            center = np.asarray(ros_detection.center),
            polygon = Polygon(np.asarray(ros_detection.polygon).reshape(-1, 3)),

            obb_3d = np.asarray(ros_detection.obb_3d).reshape(-1, 3),

            parent_frame = ros_detection.parent_frame,
            table_name = ros_detection.table_name,
            tf_name = ros_detection.tf_name,
        )
        # print("detection.box", detection.box)
        detections.append(detection)

    return detections


def gaps_to_ros(gaps):
    if gaps is None:
        return []
    
    ros_gaps = []
    for gap in gaps:
        ros_gap = ROSGap(
            id = gap.id,
            from_tf=gap.from_tf,
            to_tf=gap.to_tf,
            obb=gap.obb.ravel().tolist(),
            # todo: obb_3d=gap.obb_3d
        )
    
        ros_gaps.append(ros_gap)
    
    return ros_gaps

def gaps_to_py(ros_gaps):
    gaps = []
    for ros_gap in ros_gaps:
        gap = Gap(
            id=ros_gap.id,
            from_tf=ros_gap.from_tf,
            to_tf=ros_gap.to_tf,
            obb=np.asarray(ros_gap.obb).reshape(-1, 2)
            # todo: obb_3d=ros_gap.obb_3d
        )
        gaps.append(gap)
    return gaps


def ros_to_str(ros_msg):
    if ros_msg is None:
        return ""

    # ! fixes here to convert arrays to lists
    if hasattr(ros_msg, 'obb_3d') and isinstance(ros_msg.obb_3d, np.ndarray):
        ros_msg.obb_3d = ros_msg.obb_3d.ravel().tolist()
    
    json_msg = message_converter.convert_ros_message_to_dictionary(ros_msg) # convert to string
    return json.dumps(json_msg)
    
def str_to_ros(action_type, str_msg, is_block=True):
    # print("str_to_ros", type(str_msg))
    if str_msg == "" or str_msg is None or action_type is None:
        return None
    
    action_type_name = Action(action_type).name
    # convert snake case to camel case
    action_type_name = ''.join(word.title() for word in action_type_name.split('_'))
    # parse action to name
    ros_msg_name = 'context_action_framework/' + action_type_name
    if is_block:
        ros_msg_name += "Block"
    else:
        ros_msg_name += "Details"
    # print("ros_msg_name", ros_msg_name)
    
    # convert string to json first
    json_msg = str_msg
    if isinstance(str_msg, str):
        json_msg = json.loads(str_msg)
    return message_converter.convert_dictionary_to_ros_message(ros_msg_name, json_msg)
