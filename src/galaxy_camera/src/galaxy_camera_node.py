#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as RosImage  # 修改此处，添加别名
from cv_bridge import CvBridge
import cv2
from gx_camera import *
import numpy as np
import gxipy as gx

gx_camera_image = None

def image_publisher():
    rospy.init_node('galaxy_camera_node', anonymous=True)
    image_pub = rospy.Publisher('camera_image', RosImage, queue_size=10)  # 修改此处，使用别名
    bridge = CvBridge()

    try:
        device_manager = gx.DeviceManager()
        cam = None
        dev_info_list = create_device(device_manager)
        if not dev_info_list:
            rospy.logerr("No camera devices found.")
            return
        cam = device_manager.open_device_by_sn(dev_info_list[0].get("sn"))
        open_device(cam)
        start_acquisition(cam)
        rospy.loginfo("Camera device opened and acquisition started.")
    except Exception as e:
        rospy.logerr(f"Error initializing camera device: {e}")
        return

    rate = rospy.Rate(80)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            numpy_image = get_image(cam)
            if numpy_image is None:
                rospy.logwarn("Failed to get image from camera.")
                continue
            gx_camera_image = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)

            ros_image = bridge.cv2_to_imgmsg(gx_camera_image, "bgr8")
            image_pub.publish(ros_image)
            rospy.logdebug("Image published successfully.")
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")
        rate.sleep()

    # 关闭设备
    try:
        cam.close_device()
        rospy.loginfo("Camera device closed.")
    except Exception as e:
        rospy.logerr(f"Error closing camera device: {e}")


if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
