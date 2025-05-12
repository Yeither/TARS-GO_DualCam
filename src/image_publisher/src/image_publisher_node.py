#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_publisher():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image0', Image, queue_size=10)
    image_pub2 = rospy.Publisher('/camera/image1', Image, queue_size=10)
    
    bridge = CvBridge()

    # 请替换为你的图片路径
    image_path = '/home/yang/PFA_radar-2025-main/images/test_image.jpg'
    try:
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            rospy.logerr("无法读取图片，请检查图片路径是否正确。")
            return
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")


        rate = rospy.Rate(60)  # 60 Hz
        while not rospy.is_shutdown():
            image_pub.publish(ros_image)
            image_pub2.publish(ros_image)
            # rospy.loginfo("图片已发布。")
            rate.sleep()
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
    