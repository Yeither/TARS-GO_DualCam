#!/usr/bin/env python 
import rospy
import math
import random
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from Stitcher import Stitcher
import time


stitcher = Stitcher()
feature = False
# 创建一个 ROS 发布者，用于发布拼接后的图像
image_pub = rospy.Publisher('/stitched_image', Image, queue_size=10)
bridge = CvBridge()

def multi_callback(img_gx, img_hk):
    try:
        gx_img = bridge.imgmsg_to_cv2(img_gx, 'bgr8')
        hk_img = bridge.imgmsg_to_cv2(img_hk, 'bgr8')
        print("同步完成！")
        
        if feature:
            final = stitcher.stitch([gx_img, hk_img], showMatches=True)
        else:
            final = stitcher.ORB_stitch_images(gx_img, hk_img)


        if (final is not None) and (feature == False):
            result = final
        elif (final is not None):
            (result, vis) = final

        if result is not None:
            # 将 OpenCV 图像转换为 ROS 的 Image 消息
            ros_image = bridge.cv2_to_imgmsg(result, "bgr8")
            # 发布拼接后的图像
            image_pub.publish(ros_image)

        cv2.waitKey(1)
        
    except CvBridgeError as e:
        rospy.logerr(f"Error converting images: {e}")

if __name__ == '__main__':
    try:
        rospy.init_node('two_TOPIC', anonymous=True)
        
        subcriber_gx = message_filters.Subscriber('/camera/image1', Image)
        subcriber_hk = message_filters.Subscriber('/camera/image0', Image)
        
        sync = message_filters.ApproximateTimeSynchronizer([subcriber_gx, subcriber_hk], 20, 0.05)
        sync.registerCallback(multi_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
