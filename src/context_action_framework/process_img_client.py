import PIL # for image type checking
import rospy
from cv_bridge import CvBridge

from context_action_framework.srv import ProcessImg
from context_action_framework.types import detections_to_py

class VisionProcessImage:
    def __init__(self,
                 service_name = 'vision/basler/process_img',
                 init_ros_node = True):
        """ Interface for UGOE's vision_pipeline service "process_img", e.g. vision/basler/process_img.
        Which takes as input an image, and returns (success, detections, labelled_img, cropped_img).

        Example init:
        >>> process_image_client = VisionProcessImage()
        Example use:
        >>> success, detections, labelled_img, cropped_img = process_image_client.call_process_img(img)
        """
        self._service_name = service_name
        self._init_ros_node = init_ros_node
        self._cv_bridge = CvBridge()

        self._ros_init()

    def _ros_init(self, timeout = 3):
        if self._init_ros_node:
            try:
                rospy.init_node("vision_process_image_client_node")
            except:
                pass
        self._process_img_srv_proxy = rospy.ServiceProxy(self._service_name, ProcessImg)
        rospy.wait_for_service(self._service_name, timeout)

    def call_process_img(self,
                         img):
        """Call the process_img service on an input image.

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
            print("Service call failed: %s"%e)
