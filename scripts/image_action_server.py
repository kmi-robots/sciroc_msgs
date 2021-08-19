#! /usr/bin/env python3

import rospy
import actionlib
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sciroc_msgs.msg
from sensor_msgs.msg import Image, PointCloud2


class ImageAction:
    # create messages that are used to publish feedback/result
    _result = sciroc_msgs.msg.PerceptionResult()

    def __init__(self, name):
        # the name of action is equivalent to the topic name
        # identifies the action in ROS
        self._action_name = name

        # setup the action server and specify the callback
        self._as = actionlib.SimpleActionServer(self._action_name, sciroc_msgs.msg.PerceptionAction,
                                                execute_cb=self.execute_cb,  auto_start=False)
        self._as.start()

        # flags to start/stop the execution
        self._mode = 0
        self._done = False

        self._count = 0
        rospy.Subscriber("/cv_camera/image_raw", Image, self.img_callback)
        self._image_pub = rospy.Publisher("img_out", Image, queue_size=1)
        self._bridge = CvBridge()

    def img_callback(self, msg):
        # Nothing to do, skip callback
        if self._mode == 0:
            return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # mode 1: flip vertically
        if self._mode == 1:
            cv_image = cv2.flip(cv_image, 0)
        # mode 2: flip horizontally
        elif self._mode == 2:
            cv_image = cv2.flip(cv_image, 1)

        try:
            self._image_pub.publish(self._bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self._count += 1
        except CvBridgeError as e:
            print(e)

        # Action done, set _done to True and reset mode
        if self._count > 10:
            self._mode = 0
            self._done = True

    def execute_cb(self, goal):
        self._mode = goal.mode
        # Take a nap while waiting for the action to complete
        while not self._done:
            rospy.sleep(0.5)
        # empty point cloud as a result
        result = sciroc_msgs.msg.PerceptionResult()
        result.crop = PointCloud2()
        self._as.set_succeeded(result)

        self._done = False


if __name__ == '__main__':
    rospy.init_node('image_action')
    server = ImageAction(rospy.get_name())
    rospy.spin()
