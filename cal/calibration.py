import threading
import time
import sys

import cv2
import numpy as np

# import gxipy as gx
# from gx_camera import *

from hik_camera import call_back_get_image, start_grab_and_get_data_size, close_and_destroy_device, set_Value, \
    get_Value, image_control
from MvImport.MvCameraControl_class import *

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QTextCursor
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QGridLayout
from PyQt5 import QtCore

connection_event = threading.Event()
#0下1上
global nConnectionNum
nConnectionNum = None

points_list=None

def hik_camera_get():
    # 获得设备信息
    global camera_image
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch:枚举设备 | en:Enum device
    # nTLayerType [IN] 枚举传输层 ，pstDevList [OUT] 设备列表
    while 1:
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            # sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")
            # sys.exit()
        else:
            print("Find %d devices!" % deviceList.nDeviceNum)
            break

    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\ngige device: [%d]" % i)
            # 输出设备名字
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)
            # 输出设备ID
            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
        # 输出USB接口的信息
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nu3v device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("user serial number: %s" % strSerialNumber)
    # 手动选择设备
    if len(sys.argv) >= 1:
        nConnectionNum = int(sys.argv[1])
    else:
        print("Please provide the device number as a command-line argument.")
        sys.exit()

    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("intput error!")
        sys.exit()
    else:
        connection_event.set()


    # ch:创建相机实例 | en:Creat Camera Object
    cam = MvCamera()

    # ch:选择设备并创建句柄 | en:Select device and create handle
    # cast(typ, val)，这个函数是为了检查val变量是typ类型的，但是这个cast函数不做检查，直接返回val
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

    print(get_Value(cam, param_type="float_value", node_name="ExposureTime"),
          get_Value(cam, param_type="float_value", node_name="Gain"),
          get_Value(cam, param_type="enum_value", node_name="TriggerMode"),
          get_Value(cam, param_type="float_value", node_name="AcquisitionFrameRate"))

    # 设置设备的一些参数
    set_Value(cam, param_type="float_value", node_name="ExposureTime", node_value=16000)  # 曝光时间
    set_Value(cam, param_type="float_value", node_name="Gain", node_value=17.9)  # 增益值
    # 开启设备取流
    start_grab_and_get_data_size(cam)
    # 主动取流方式抓取图像
    stParam = MVCC_INTVALUE_EX()

    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
    ret = cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()
    nDataSize = stParam.nCurValue
    pData = (c_ubyte * nDataSize)()
    stFrameInfo = MV_FRAME_OUT_INFO_EX()

    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    while True:
        ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            image = np.asarray(pData)
            # 处理海康相机的图像格式为OPENCV处理的格式
            camera_image = image_control(data=image, stFrameInfo=stFrameInfo)
        else:
            print("no data[0x%x]" % ret)


def gx_camera_get():
    global camera_image
    device_manager = gx.DeviceManager()
    cam = None
    dev_info_list = create_device(device_manager)
    cam = device_manager.open_device_by_sn(dev_info_list[0].get("sn"))
    open_device(cam)
    start_acquisition(cam)

    while True:
        numpy_image = get_image(cam)
        camera_image = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)


def video_capture_get():
    global camera_image
    cam = cv2.VideoCapture(1)
    while True:
        ret, img = cam.read()
        if ret:
            camera_image = img
            time.sleep(0.016)  # 60fps


def video_test_get():
    global camera_image
    video = cv2.VideoCapture('/home/mumu/Videos/test1.avi')
    while video.isOpened():
        ret, frame = video.read()
        if not ret: 
            break
        camera_image = frame
        time.sleep(0.016)
    video.release()


color = [(0, 255, 0), (255, 0, 0)]


class MyUI(QWidget):
    def __init__(self):
        super().__init__()
        self.capturing = True
        self.initUI()

    def initUI(self):
        # 左上角部分
        self.state = state
        self.left_top_label = QLabel(self)
        self.left_top_label.setFixedSize(1350, 1000)
        self.left_top_label.setStyleSheet("border: 2px solid black;")
        self.left_top_label.mousePressEvent = self.left_top_clicked
        self.image_points = [[(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)]]
        # if nConnectionNum == 0:
        #     self.map_points = [[(668, 2199), (859, 2199), (684, 1897), (839, 1897)], [(278, 1132), (280, 1543), (1126, 1263), (1126, 1620)]]
        # elif nConnectionNum == 1:
        #     self.map_points = [[(668, 724), (861, 721), (684, 1897), (842, 1897)], [(474, 1182), (597, 1680), (1129, 1256), (1126, 1617)]]
        if nConnectionNum == 0:
            self.map_points = [[(657, 2202), (850, 2199), (676, 1894), (829, 1894)], [(272, 1135), (270, 1546), (1112, 1266), (1115, 1627)]]
        elif nConnectionNum == 1:
            self.map_points = [[(668, 724), (861, 721), (684, 1897), (842, 1897)], [(474, 1182), (597, 1680), (1129, 1256), (1126, 1617)]]
        
        self.image_count = 0
        self.map_count = 0
        # 右上角部分
        self.right_top_label = QLabel(self)
        self.right_top_label.setFixedSize(550, 900)
        self.right_top_label.setStyleSheet("border: 2px solid black;")
        self.right_top_label.mousePressEvent = self.right_top_clicked

        # 左下角部分
        self.left_bottom_text = QTextEdit(self)
        self.left_bottom_text.setFixedSize(300, 60)

        # 右下角部分
        self.button1 = QPushButton('开始标定', self)
        self.button1.setFixedSize(100, 30)
        self.button1.clicked.connect(self.button1_clicked)

        self.button2 = QPushButton('切换高度', self)
        self.button2.setFixedSize(100, 30)
        self.button2.clicked.connect(self.button2_clicked)

        self.button3 = QPushButton('加载坐标', self)
        self.button3.setFixedSize(100, 30)
        self.button3.clicked.connect(self.button3_clicked)

        self.button4 = QPushButton('保存计算', self)
        self.button4.setFixedSize(100, 30)
        self.button4.clicked.connect(self.button4_clicked)
        self.height = 0
        self.T = []
        #  /home/yang/PFA_radar-2025-main
        if self.state == 'R':
            if nConnectionNum == 0:
                self.save_path = '/home/yang/double_camera/src/detect/scripts/npy/arrays_test_red.npy'
                self.save_path2 = '/home/yang/PFA_radar-2025-main/arrays_test_red.npy'
                right_image_path = "/home/yang/double_camera/cal/images/map_red_marked_close.jpg"
                # right_image_path = "/home/yang/double_camera/cal/images/map_red.jpg"
                print("红方：近场")
            elif nConnectionNum == 1:
                self.save_path = '/home/yang/double_camera/src/detect2/scripts/npy/arrays_test_red.npy'
                self.save_path2 = self.save_path
                # self.save_path = '/home/yang/PFA_radar-2025-main/arrays_test_red.npy'
                # right_image_path = "/home/yang/double_camera/cal/images/map_red.jpg"
                right_image_path = "/home/yang/double_camera/cal/images/map_red_marked_far.jpg"  # 替换为右边图片的路径
                print("红方：远场")
        else:
            if nConnectionNum == 0:
                self.save_path = '/home/yang/double_camera/src/detect/scripts/npy/arrays_test_blue.npy'
                self.save_path2 = '/home/yang/PFA_radar-2025-main/arrays_test_blue.npy'
                right_image_path = "/home/yang/double_camera/cal/images/map_blue_marked_close.jpg" 
                # right_image_path = "/home/yang/map_blue.jpg" 
                # right_image_path = "/home/yang/double_camera/cal/images/map_blue.jpg"
                print("蓝方：近场")
            elif nConnectionNum == 1:
                self.save_path = '/home/yang/double_camera/src/detect2/scripts/npy/arrays_test_blue.npy'
                self.save_path2 = self.save_path
                # self.save_path = '/home/yang/PFA_radar-2025-main/arrays_test_blue.npy'
                right_image_path = "/home/yang/double_camera/cal/images/map_blue_marked_far.jpg"  # 替换为右边图片的路径
                # right_image_path = "/home/yang/double_camera/cal/images/map_blue.jpg"
                # right_image_path = "/home/yang/map_blue.jpg" 
                print("蓝方：远场")

        # _,left_image = self.camera_capture.read()
        left_image = camera_image
        print("self.state :",self.state ,"\nnConnectionNum:",nConnectionNum)
        right_image = cv2.imread(right_image_path)

        # 记录缩放比例
        self.left_scale_x = left_image.shape[1] / 1350.0
        self.left_scale_y = left_image.shape[0] / 1000.0

        self.right_scale_x = right_image.shape[1] / 550.0
        self.right_scale_y = right_image.shape[0] / 900.0
        left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
        self.left_image = cv2.resize(left_image, (1350, 1000))
        right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)
        self.right_image = cv2.resize(right_image, (550, 900))
        # 缩放图像
        self.update_images()

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.start(50)  # 50毫秒更新一次相机
        # 设置按钮样式
        self.set_button_style(self.button1)
        self.set_button_style(self.button2)
        self.set_button_style(self.button3)
        self.set_button_style(self.button4)

        grid_layout = QGridLayout()
        grid_layout.addWidget(self.button1, 0, 0)
        grid_layout.addWidget(self.button2, 0, 1)
        grid_layout.addWidget(self.button3, 1, 0)
        grid_layout.addWidget(self.button4, 1, 1)

        buttons_and_text_widget = QWidget()

        hbox_buttons_and_text = QHBoxLayout(buttons_and_text_widget)
        hbox_buttons_and_text.addLayout(grid_layout)
        hbox_buttons_and_text.addWidget(self.left_bottom_text)

        vbox_left = QVBoxLayout()
        vbox_left.addWidget(self.left_top_label)

        vbox_right = QVBoxLayout()
        vbox_right.addWidget(self.right_top_label)
        vbox_right.addWidget(buttons_and_text_widget)

        hbox = QHBoxLayout()
        hbox.addLayout(vbox_left)
        hbox.addLayout(vbox_right)

        self.setLayout(hbox)
        self.setGeometry(0, 0, 1900, 1000)
        self.setWindowTitle('Calibration UI')
        self.show()
    
    def harris_features(self, image):
        try:
            # 转换为灰度图
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = np.float32(gray)  # Harris算法需要float32类型
            
            # 应用Harris角点检测
            dst = cv2.cornerHarris(gray, blockSize=2, ksize=1, k=0.001)
            
            # 角点增强（膨胀操作）
            dst = cv2.dilate(dst, None)
            
            # 设置角点检测阈值（可根据实际情况调整）
            threshold = 0.01 * dst.max()
            corner_mask = dst > threshold
            
            # 创建角点列表
            kp = []
            for y in range(corner_mask.shape[0]):
                for x in range(corner_mask.shape[1]):
                    if corner_mask[y, x]:
                        # 创建KeyPoint对象（x, y坐标，尺寸，方向）
                        kp.append(cv2.KeyPoint(x, y, 10))  # 尺寸设为10，方向默认为-1（未定义）
            
            # 在原图上绘制角点
            image_with_keypoints = cv2.drawKeypoints(
                image, kp, None, color=(0, 255, 0), 
                flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS
            )
            
            # 返回结果（注意：Harris没有描述符，所以第二个返回值为None）
            return image_with_keypoints, kp
        
        except Exception as e:
            print(f"发生错误: {e}")
            return None, None

    def sift_features(self, image):
        try:
            # 转换为灰度图
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 初始化SIFT检测器
            sift = cv2.SIFT_create()
            kp, des = sift.detectAndCompute(gray, None)         
            # 在原图上绘制关键点
            image_with_keypoints = cv2.drawKeypoints(image, kp, None, color=(0, 255, 0), flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
            
            return image_with_keypoints,kp
        except Exception as e:
            print(f"发生错误: {e}")
            return None,None
    
    def fast_features(self, image):
        try:
            # 转换为灰度图
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 初始化FAST检测器
            # 参数说明：
            #   threshold: 像素强度差异阈值，用于判断是否为角点
            #   nonmaxSuppression: 是否应用非极大值抑制，避免角点聚集
            #   type: 角点检测类型，可选值包括：
            #         - cv2.FAST_FEATURE_DETECTOR_TYPE_5_8 (默认)
            #         - cv2.FAST_FEATURE_DETECTOR_TYPE_7_12
            #         - cv2.FAST_FEATURE_DETECTOR_TYPE_9_16
            fast = cv2.FastFeatureDetector_create(
                threshold=10, 
                nonmaxSuppression=True,
                type=cv2.FAST_FEATURE_DETECTOR_TYPE_9_16
            )
            
            # 检测关键点
            kp = fast.detect(gray, None)
            
            # 在原图上绘制关键点
            image_with_keypoints = cv2.drawKeypoints(
                image, kp, None, color=(0, 255, 0),
                flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS
            )
            
            # 返回结果（注意：FAST不计算描述符，所以第二个返回值为None）
            return image_with_keypoints, kp
        
        except Exception as e:
            print(f"发生错误: {e}")
            return None, None

    def fast_orb_features(self, image):
        try:
            # 转换为灰度图
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 初始化ORB检测器
            # 参数说明：
            #   nfeatures: 保留的最大特征点数量
            #   scaleFactor: 金字塔缩放因子
            #   nlevels: 金字塔层数
            #   edgeThreshold: 关键点检测的边界阈值
            #   firstLevel: 金字塔的第一层索引
            #   WTA_K: 生成BRIEF描述子时使用的随机点对数量
            #   scoreType: 关键点排序方法（HARRIS_SCORE或FAST_SCORE）
            #   patchSize: 描述符计算的补丁大小
            #   fastThreshold: FAST算法的阈值
            orb = cv2.ORB_create(
                nfeatures=1000,
                scaleFactor=1.2,
                nlevels=8,
                edgeThreshold=31,
                firstLevel=0,
                WTA_K=2,
                scoreType=cv2.ORB_HARRIS_SCORE,
                patchSize=31,
                fastThreshold=20
            )
            
            # 检测关键点并计算描述符
            kp, des = orb.detectAndCompute(gray, None)
            
            # 在原图上绘制关键点
            image_with_keypoints = cv2.drawKeypoints(
                image, kp, None, color=(0, 255, 0),
                flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS
            )
            
            # 返回结果（ORB同时提供关键点和描述符）
            return image_with_keypoints, kp
        
        except Exception as e:
            print(f"发生错误: {e}")
            return None, None


    def find_nearest_point(self, point, point_list):
        if not point_list:
            return None, -1     
        # 将输入点转换为NumPy数组以便计算
        point = np.array(point)
        # 计算给定点到point_list中每个点的欧氏距离
        distances = np.sqrt(np.sum((np.array(point_list) - point) ** 2, axis=1))
        # 找出最小距离的索引
        nearest_index = np.argmin(distances)
        # 返回最近点及其索引
        return point_list[nearest_index], nearest_index

    def keyPressEvent(self, event):
        # 按下键盘事件
        if event.key() == Qt.Key_Escape:
            self.close()

    def update_images(self):
        left_pixmap = self.convert_cvimage_to_pixmap(self.left_image)
        self.left_top_label.setPixmap(left_pixmap)

        right_pixmap = self.convert_cvimage_to_pixmap(self.right_image)
        self.right_top_label.setPixmap(right_pixmap)

    def update_camera(self):
        global points_list
        if self.capturing:
            img0 = camera_image
            
            left_image,kp = self.fast_features(img0)
            points_list = [(kp.pt[0], kp.pt[1]) for kp in kp]
            left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
            self.left_image = cv2.resize(left_image, (1350, 1000))
            self.update_images()

    def left_top_clicked(self, event):
        # 图像点击事件 - 仅处理鼠标左键点击
        if not self.capturing and event.button() == QtCore.Qt.LeftButton:  # 添加左键判断
            x = int(event.pos().x() * self.left_scale_x)
            y = int(event.pos().y() * self.left_scale_y)

            click_point = (int(x), int(y))
            point, id = self.find_nearest_point(click_point, points_list)
            self.image_points[self.height][self.image_count % 4] = (point[0], point[1])
            point = (int(point[0]/self.left_scale_x), int(point[1]/self.left_scale_y))

            cv2.circle(self.left_image, point, 4, color[self.height], -1)
            cv2.putText(self.left_image, str(self.image_count % 4),
                        (int(x / self.left_scale_x), int(y / self.left_scale_y)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color[self.height], 3)
            self.image_count += 1
            self.update_images()
            self.append_text(f'图像真实点击坐标：({x}, {y})')
        elif not self.capturing and event.button() == QtCore.Qt.RightButton:  # 添加左键判断
            x = int(event.pos().x() * self.left_scale_x)
            y = int(event.pos().y() * self.left_scale_y)

            point = (int(x), int(y))
            
            self.image_points[self.height][self.image_count % 4] = (point[0], point[1])
            point = (int(point[0]/self.left_scale_x), int(point[1]/self.left_scale_y))

            cv2.circle(self.left_image, point, 4, color[self.height], -1)
            cv2.putText(self.left_image, str(self.image_count % 4),
                        (int(x / self.left_scale_x), int(y / self.left_scale_y)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color[self.height], 3)
            self.image_count += 1
            self.update_images()
            self.append_text(f'图像真实点击坐标：({x}, {y})')


    def right_top_clicked(self, event):
        # 地图点击事件
        if not self.capturing:
            x = int(event.pos().x() * self.right_scale_x)
            y = int(event.pos().y() * self.right_scale_y)
            self.map_points[self.height][self.map_count % 4] = (x, y)

            cv2.circle(self.right_image, (int(x / self.right_scale_x), int(y / self.right_scale_y)), 4,
                        color[self.height],-1)
            cv2.putText(self.right_image, str(self.map_count % 4),
                        (int(x / self.right_scale_x), int(y / self.right_scale_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color[self.height], 2)
            self.map_count += 1
            self.update_images()
            self.append_text(f'地图真实点击坐标：({x}, {y})')

    def button1_clicked(self):
        # 按钮1点击事件
        self.append_text('开始标定')
        self.capturing = False

        print('开始标定')

    def button2_clicked(self):
        # 按钮2点击事件
        self.append_text('切换高度')
        self.image_count = 0
        self.map_count = 0
        self.height = (self.height + 1) % 3
        print('切换高度')

    def button3_clicked(self):
        # 按钮3点击事件
        self.append_text('加载坐标')  # 该功能还未制作

        print('加载坐标')

    def button4_clicked(self):
        # 按钮4点击事件
        print(self.image_points)
        print(self.map_points)
        for i in range(0, 2):
            image_point = np.array(self.image_points[i], dtype=np.float32)
            map_point = np.array(self.map_points[i], dtype=np.float32)
            self.T.append(cv2.getPerspectiveTransform(image_point, map_point))

        np.save("./image_points.txt", self.image_points)
        np.save("./map_points.txt", self.map_points)
        np.save(self.save_path, self.T)
        np.save(self.save_path2, self.T)

        self.append_text('保存计算')
        print('保存计算', self.save_path)
        print('结果：',self.T)
        time.sleep(1)
        sys.exit()

    def convert_cvimage_to_pixmap(self, cvimage):
        height, width, channel = cvimage.shape
        bytes_per_line = 3 * width
        qimage = QImage(cvimage.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap

    def set_button_style(self, button):
        button.setStyleSheet("QPushButton { font-size: 18px; }")

    def append_text(self, text):
        # 在文本组件中追加文本
        current_text = self.left_bottom_text.toPlainText()
        self.left_bottom_text.setPlainText(current_text + '\n' + text)
        # 自动向下滚动文本组件
        cursor = self.left_bottom_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.left_bottom_text.setTextCursor(cursor)


if __name__ == '__main__':
    camera_mode = 'hik'  # 'test':图片测试, 'video':视频测试, 'hik':海康相机, 'galaxy':大恒相机, 'usb':USB相机
    camera_image = None
    state = 'B'  # R:红方/B:蓝方

    if len(sys.argv) > 0:
        nConnectionNum = int(sys.argv[1])
    
    if camera_mode == 'test':
        camera_image = cv2.imread('/home/yang/PFA_radar-2025-main/images/test_image.jpg')
    elif camera_mode == 'video':
        thread_camera = threading.Thread(target=video_test_get, daemon=True)
        thread_camera.start()
    elif camera_mode == 'usb':
        thread_camera = threading.Thread(target=video_capture_get, daemon=True)
        thread_camera.start()
    elif camera_mode == 'hik':
        thread_camera = threading.Thread(target=hik_camera_get, daemon=True)
        thread_camera.start()
    elif camera_mode == 'galaxy':
        thread_camera = threading.Thread(target=gx_camera_get, daemon=True)
        thread_camera.start()

    while camera_image is None:
        print("等待图像。。。")
        time.sleep(0.5)
    if camera_mode !="test":
        connection_event.wait()
    print("nConnectionNum get the number :",nConnectionNum)
    app = QApplication(sys.argv)
    myui = MyUI()
    sys.exit(app.exec_())
