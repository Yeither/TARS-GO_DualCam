#!/usr/bin/env python 
from Stitcher import Stitcher
import cv2
import time

# 读取拼接图片
imageA = cv2.imread("/home/yang/Pictures/up.png")
imageB = cv2.imread("/home/yang/Pictures/down.png")
# 把图片拼接成全景图
stitcher = Stitcher()
a = time.time()
(result, vis) = stitcher.stitch([imageA, imageB], showMatches=True)
b = time.time()
print("a-b=",a-b)
# 显示所有图片
cv2.imshow("Image A", imageA)
cv2.imshow("Image B", imageB)
cv2.imshow("Keypoint Matches", vis)
cv2.imshow("Result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
