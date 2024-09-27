import rospy
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from typing import Optional
import PIL # for image type checking
from context_action_framework.msg import Detection, Detections
from context_action_framework.srv import ProcessImg
from context_action_framework.types import Label, detections_to_py
import time
from abc import ABC, abstractmethod
from typing import Union, List

class BaseVisionInterface(ABC):
    def __init__(self):
        0
    @abstractmethod
    def get_detections(self,
                       desired_class: Union[None, Label, List[Label]],
                       timeout: Union[float, int]):
        """ Return detections from the vision system.
        Args:
        desired_class: Union[None, str, List[str]]. If None, return all detections. If (hca_back), return all detections of the specific class.
        timeout: float: Maximum amount of time to wait for detections"""

class VisionInterface(BaseVisionInterface):
    def __init__(self,
                 vision_topic = '/vision/basler/detections',
                 activate_vision_service_topic = '/vision/basler/enable',
                 process_img_service_topic = 'vision/basler/process_img',
                 run_mode = 'topic',
                 init_ros_node = True):
        """ Class that enables receiving information from the Vision System, and sending an image to it & receiving detections using the process_img function
    
        Args:
        vision_topic: str
            full vision topic for detections that will be subscribed to.
        run_mode: str, one of ['topic', 'service', 'flexbe']
            Determines whether to initialize ROS subscribers.

        Methods:

        - process_img: PIL.Image.Image: takes as input an image, and returns (success, detections, labelled_img, cropped_img).

        Examples:
        >>> vision_interface = VisionInterface(vision_topic = '/vision/basler/detections',
                             activate_vision_service_topic = '/vision/basler/enable')

        >>> success, detections, labelled_img, cropped_img = vision_interface.call_process_img(img)
        """

        self._vision_topic = vision_topic
        self._activate_vision_service_topic = activate_vision_service_topic
        self._process_img_service_topic = process_img_service_topic
        self._run_mode = run_mode
        self._init_ros_node = init_ros_node

        self._enable_camera_svc = None # By default no such service will exist
        self._cv_bridge = CvBridge()
        self._detections = None

        if run_mode not in ['topic', 'service', 'flexbe']:
            raise ValueError(f"run_mode must be one of {allowed_run_modes}.")

        if self._run_mode == 'topic':
            self._ros_init()
        elif self._run_mode == 'flexbe' or self.run_mode == 'manual':
            rospy.loginfo("not running in topic mode; manually update detections!")
        else: 
            raise Exception("Running in service mode not yet supported!")
            
    def _ros_init(self, timeout = 3):
        """ Initializes ROS detections subscriber and ROS enable camera ServiceProxy. """
        if self._init_ros_node:
            try:
                rospy.init_node("vision_interface_nove")
            except:
                pass

        rospy.loginfo("VisionUtils listening to: {}".format(self._vision_topic))
        #self._detections_sub = rospy.Subscriber(self._vision_topic, Detections, self._process_detections)
        self._enable_camera_svc = rospy.ServiceProxy(self._activate_vision_service_topic, SetBool)
        self._process_img_srv_proxy = rospy.ServiceProxy(self._process_img_service_topic, ProcessImg)
        rospy.wait_for_service(self._process_img_service_topic, timeout)

    def call_process_img(self, img):
        """Call the process_img service on an input image, to get back detections and cropped image of detected object.

        Input args:
        img: PIL Image or cv2 image
        timeout: float or int: timeout in which to wait for service

        Returns:
        success, detections, labelled_img, cropped_img

        Example call:
        >>> process_image_client = VisionProcessImage()
        >>> success, detections, labelled_img, cropped_img = process_image_client.call_process_img(img)
        """

        # Convert to cv2 if img is type PIL.Image.Image
        if isinstance(img, PIL.Image.Image):
            img = np.array(img)

        imgmsg = self._cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
        try:
            response = self._process_img_srv_proxy(imgmsg)
            detections = detections_to_py(response.detections)
            labelled_img = self._cv_bridge.imgmsg_to_cv2(response.labelled_image, desired_encoding='passthrough')
            cropped_img = self._cv_bridge.imgmsg_to_cv2(response.cropped_image, desired_encoding='passthrough')

            return response.success, detections, labelled_img, cropped_img
            # return response
        except rospy.ServiceException as e:
            print("VisionInterface: Service call failed: %s"%e)

    def get_detections(self, desired_class: Label = None, timeout = 5.0):
        """ 
        Function that calls vision_utils.update_detections and allows for a number of retries.

        Args:
        desired_class: If not None, return ALL detections of this class. If None, return all detections
        timeout: timeout value in seconds.

        Returns:

        detections only of particular class, if desired_class is not None
        all detections, if desired_class is None. May return empty list if message received but no objects detected.
        None, if no detections message is received.

        Example call:
        >>> detections = vision_interface.get_detections(desired_class = Label.smoke_detector, timeout = 3)
        """
        DEFAULT_WAIT_TIME_FOR_MESSAGE = 2 # rospy.wait_for_message timeout. If this is too low (in regards to vision system FPS), we will never receive a message within specified time

        received_message = 0
        success = 0 # Keep track of 1. whether we received a message, or 2. found a desired object

        self._detections = None
        start_t = time.time()
        while (success==0) and ((time.time() - start_t) < timeout):
            try:
                self._update_detections(timeout=DEFAULT_WAIT_TIME_FOR_MESSAGE)

                if (desired_class is None) and (len(self._detections)>0):
                    success = 1
                    return self._detections
                else:
                    # Even if we successfully get detections msg, check that at least one item of desired class if found. Else, keep looking. 
                    #detections = self.get_particular_class_from_detections(desired_class = desired_class)
                    detections = [i for i in self._detections if i.label == desired_class]
                    if len(detections) > 0:
                        success = 1
                        return detections
            except Exception as e:
                0

        return None # None if no message received or no desired class detections available

    def clear_detections(self):
        """ Clear historical detections (self._detections) and replace with empty list. """
        self._detections = []

    def enable_camera(self, enable = True):
        if self._enable_camera_svc is None: return 0

        if enable not in [True, False]:
            raise ValueError("enable_camera argument should be a boolean type")
        self._enable_camera_svc.call(enable)

    def detections_to_description(self, detections = None):
        """ Takes the latest vision detections and transforms then into a user-friendly textual description of each
        detected object. Useful for LLM."""

        if detections is None: detections = self._detections
        description = ''
        for index, obj in enumerate(detections):
            DisassemblyObject = DisassemblyObject(obj)
            #description += f"{index}. Object {Label(obj.label).name} at world coordinates x = [{obj.tf.translation.x} {obj.tf.translation.y} {obj.tf.translation.z}]\n"
            description += f"{index}. [{DisassemblyObject.general_class}].\n"
        return description

    def _update_detections(self, timeout = 0.5):
        """ Wait for ros Detections message within specified timeout.
        Args:
        timeout: float
            Duration of time to wait to receive a message.

        Calls downstream function self._process_detections. End result is self.detections object."""
        data = rospy.wait_for_message(self._vision_topic, Detections, timeout = timeout)
        self._process_detections(data)

    def _process_detections(self, data):
        """ Takes as input vision detections and modify/update them with useful information."""
        #self._detections = None
        if len(data.detections) > 0:
            disassembly_objects = []
            #self.detections = [detections_to_py(detection) for detection in data.detections]
            detections = detections_to_py(data.detections)
            self._detections = detections
