#!/usr/bin/env python3.10

import threading
import time
from collections import deque
import serial

import sys

from detect2.msg import Serial_Send_Up
sys.path.insert(0, "/home/yang/double_camera/src/detect2/scripts") 


import cv2
import numpy as np
from detect_function import YOLOv5Detector
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# ROS节点初始化
rospy.init_node('main_image_listener1', anonymous=True)

# yaml读取
state =  rospy.get_param("/state")   # R：红方；B：蓝方
npy_path = rospy.get_param("/device_up/image/npy")
map_v_path = rospy.get_param("/device_up/image/map_v")
mask_map_path = rospy.get_param("/device_up/image/mask_map")
hide_map_path = rospy.get_param("/device_up/image/hide_map")
map_path = rospy.get_param("/device_up/image/map")
car_engine_path = rospy.get_param("/device_up/detect/car/engine")
car_yaml_path = rospy.get_param("/device_up/detect/car/yaml")
armor_engine_path = rospy.get_param("/device_up/detect/armor/engine")
armor_yaml_path = rospy.get_param("/device_up/detect/armor/yaml")

print("*"*10,"config","*"*10)
print('state              :', state)
print('npy_path           :', npy_path)
print('map_v_path         :', map_v_path)
print('mask_map_path      :', mask_map_path)
print('hide_map_path      :', hide_map_path)
print('map_path           :', map_path)
print('car_engine_path    :', car_engine_path)
print('car_yaml_path      :', car_yaml_path)
print('armor_engine_path  :', armor_engine_path)
print('armor_yaml_path    :', armor_yaml_path)
print("*"*26)

loaded_arrays = np.load(npy_path)  # 加载标定好的仿射变换矩阵
map_image = cv2.imread(map_v_path)  # 加载红方视角地图
mask_image = cv2.imread(mask_map_path)  # 加载红发落点判断掩码
hide_mask = cv2.imread(hide_map_path)

# 导入战场每个高度的不同仿射变化矩阵
M_height_r = loaded_arrays[1]  # R型高地
M_height_g = loaded_arrays[1]  # 环形高地
M_ground = loaded_arrays[0]  # 地面层、公路层

# 确定地图画面像素，保证不会溢出
height, width = mask_image.shape[:2]
height -= 1
width -= 1


if_guess = [False,False,False,False,False] # 欣姐说我得给哨兵数据

# 加载战场地图
map_backup = cv2.imread(map_path)
map = map_backup.copy()

# 盲区预测次数
guess_value = {
    "B1": 0,
    "B2": 0,
    "B3": 0,
    "B4": 0,
    "B5": 0,
    "B7": 0,
    "R1": 0,
    "R2": 0,
    "R3": 0,
    "R4": 0,
    "R5": 0,
    "R7": 0
}

# 当前标记进度（用于判断是否预测正确正确）
mark_progress = {
    "B1": 0,
    "B2": 0,
    "B3": 0,
    "B4": 0,
    "B5": 0,
    "B7": 0,
    "R1": 0,
    "R2": 0,
    "R3": 0,
    "R4": 0,
    "R5": 0,
    "R7": 0
}

# 机器人名字对应ID
mapping_table = {
    "R1": 1,
    "R2": 2,
    "R3": 3,
    "R4": 4,
    "R5": 5,
    "R6": 6,
    "R7": 7,
    "B1": 101,
    "B2": 102,
    "B3": 103,
    "B4": 104,
    "B5": 105,
    "B6": 106,
    "B7": 107
}


# 预测点索引
guess_index = {
    'B1': 0,
    'B2': 0,
    'B3': 0,
    'B4': 0,
    'B5': 0,
    'B7': 0,
    'R1': 0,
    'R2': 0,
    'R3': 0,
    'R4': 0,
    'R5': 0,
    'R7': 0,
}

# 哨兵
# 工程：兑换站
# 英雄：
# 中央高地
guess_table_B = {
    "G0": [(662,752), (408,754)],           # 堡垒，家
    "G1": [(216, 92), (1344, 798)],        # 兑换站，大风车
    "G2": [(1010,1066), (1000,404)],           # 洞，前哨站洞
    "G3": [(248, 252), (1068, 1126)]        # 维修站，洞
}

guess_table_R = {
    "G0": [(2128, 754),(2388, 754)],
    "G1": [(2582, 1398), (1474, 700)],
    "G2": [(1772, 442), (1790, 1108)],
    "G3": [(2544, 1254), (1736, 392)]
}

target_position = [[0.0,0.0], [0.0,0.0], [0.0,0.0], [0.0,0.0], [0.0,0.0], [0.0,0.0]]

# 串口发送线程
def ser_send(pub):
    seq = 0
    global guess_value

    target_position_ROS = Serial_Send_Up()

    def return_xy_B(send_name):
        # 转换为地图坐标系
        filtered_xyz = (2800 - all_filter_data[send_name][1], all_filter_data[send_name][0])
        # 转换为裁判系统单位 cm
        ser_x = int(filtered_xyz[0])
        ser_y = int(1500 - filtered_xyz[1])
        return [ser_x, ser_y]
    
    def return_xy_R(send_name):
        # 转换为地图坐标系
        filtered_xyz = (all_filter_data[send_name][1], 1500 - all_filter_data[send_name][0])
        # 转换为裁判系统单位M
        ser_x = int(filtered_xyz[0])
        ser_y = int(1500 - filtered_xyz[1])
        return [ser_x, ser_y]

    def send_point(target_position,if_guess):
        try:
            target_position_ROS.position_1_up =target_position[0]
            target_position_ROS.position_2_up =target_position[1]
            target_position_ROS.position_3_up =target_position[2]
            target_position_ROS.position_4_up =target_position[3]
            target_position_ROS.position_7_up =target_position[5]
            target_position_ROS.if_guess = if_guess
            pub.publish(target_position_ROS)
            return True
        except Exception as r:
            # import traceback
            # traceback.print_exc()
            print('send_point2 错误 %s' % (r))
            time.sleep(0.5)
            return False


    def get_guess_point(send_name):
        guess_point = [0,0]
        if state == 'B':
            if send_name == "R7":
                if guess_value[send_name] == 0:
                    guess_point = guess_table_B["G0"][0]
                    guess_value[send_name] = 1
                else:
                    guess_point = guess_table_B["G0"][1]
                    guess_value[send_name] = 0
            elif send_name == "R1":
                if guess_value[send_name] == 0:
                    guess_point = guess_table_B["G2"][0]
                    guess_value[send_name] = 1
                else:
                    guess_point = guess_table_B["G2"][1]
                    guess_value[send_name] = 0
            elif send_name == "R2":
                if guess_index[send_name] == 1:
                    guess_point = guess_table_B["G1"][0]
                else:
                    guess_point = guess_table_B["G1"][1]
            else:
                if guess_index[send_name] == 1:
                    guess_point = guess_table_B["G3"][1]
                else:
                    guess_point = guess_table_B["G3"][0]

        if state == 'R':
            if send_name == "B7":
                if guess_value[send_name] == 0:
                    guess_point = guess_table_R["G0"][0]
                    guess_value[send_name] = 1
                else:
                    guess_point = guess_table_R["G0"][1]
                    guess_value[send_name] = 0
            elif send_name == "B1":
                if guess_value[send_name] == 0:
                    guess_point = guess_table_R["G2"][0]
                    guess_value[send_name] = 1
                else:
                    guess_point = guess_table_R["G2"][1]
                    guess_value[send_name] = 0
            elif send_name == "B2":
                if guess_index[send_name] == 1:
                    guess_point = guess_table_R["G1"][0]
                else:
                    guess_point = guess_table_R["G1"][1]
            else:
                if guess_index[send_name] == 1:
                    guess_point = guess_table_R["G3"][1]
                else:
                    guess_point = guess_table_R["G3"][0]
        return guess_point


    while (True):
        try:
            all_filter_data = filter.get_all_data()
            
            if state == 'R':
                # 英雄
                if all_filter_data.get('B1', False):
                    target_position[0] = return_xy_B('B1')
                    if_guess[0] = False
                else:
                    target_position[0] = get_guess_point("B1")
                    if_guess[0] = True
                # 工程
                if all_filter_data.get('B2', False):
                    target_position[1] = return_xy_B('B2')
                    if_guess[1] = False
                else:
                    target_position[1] = get_guess_point("B2")
                    if_guess[1] = True
                # 步兵3号
                if all_filter_data.get('B3', False):
                    target_position[2] = return_xy_B('B3')
                    if_guess[2] = False
                else:
                    target_position[2] = get_guess_point("B3")
                    if_guess[2] = True
                # 步兵4号
                if all_filter_data.get('B4', False):
                    target_position[3] = return_xy_B('B4')
                    if_guess[3] = False
                else:
                    target_position[3] = get_guess_point("B4")
                    if_guess[3] = True
                # 步兵5号
                if all_filter_data.get('B5', False):
                    target_position[4] = return_xy_B('B5')
                else:
                    target_position[4] = get_guess_point("B5")
                # 哨兵
                if all_filter_data.get('B7', False):
                    target_position[5] = return_xy_B('B7')
                    if_guess[4] = False
                else:
                    target_position[5] = get_guess_point("B7")
                    if_guess[4] = True
                seq = send_point(target_position,if_guess)

            if state == 'B':
                # 英雄
                if all_filter_data.get('R1', False):
                    target_position[0] = return_xy_R('R1')
                    if_guess[0] = False
                else:
                    target_position[0] = get_guess_point("R1")
                    if_guess[0] = True
                # 工程
                if all_filter_data.get('R2', False):
                    target_position[1] = return_xy_R('R2')
                    if_guess[1] = False
                else:
                    target_position[1] = get_guess_point("R2")
                    if_guess[1] = True
                # 步兵3号
                if all_filter_data.get('R3', False):
                    target_position[2] = return_xy_R('R3')
                    if_guess[2] = False
                else:
                    target_position[2] = get_guess_point("R3")
                    if_guess[2] = True
                # 步兵4号
                if all_filter_data.get('R4', False):
                    target_position[3] = return_xy_R('R4')
                    if_guess[3] = False
                else:
                    target_position[3] = get_guess_point("R4")
                    if_guess[3] = True
                # 步兵5号
                if all_filter_data.get('R5', False):
                    target_position[4] = return_xy_R('R5')
                else:
                    target_position[4] = get_guess_point("R5")
                # 哨兵
                if all_filter_data.get('R7', False):
                    target_position[5] = return_xy_R('R7')
                    if_guess[4] = False
                else:
                    target_position[5] = get_guess_point("R7")
                    if_guess[4] = True
                seq = send_point(target_position,if_guess)
        
            time.sleep(0.333)
        except Exception as r:
            import traceback
            traceback.print_exc()
            print('未知错误 %s' % (r))
            time.sleep(2)


# 机器人坐标滤波器（滑动窗口均值滤波）
class Filter:
    def __init__(self, window_size, max_inactive_time=2.0):
        self.window_size = window_size
        self.max_inactive_time = max_inactive_time
        self.data = {}  # 存储不同机器人的数据
        self.window = {}  # 存储滑动窗口内的数据
        self.last_update = {}  # 存储每个机器人的最后更新时间

    # 添加机器人坐标数据
    def add_data(self, name, x, y, threshold=100000.0):  # 阈值单位为mm，实测没啥用，不如直接给大点
        # global guess_list
        if name not in self.data:
            # 如果实体名称不在数据字典中，初始化相应的deque。
            self.data[name] = deque(maxlen=self.window_size)
            self.window[name] = deque(maxlen=self.window_size)

        if len(self.window[name]) >= 2:
            # 计算当前坐标与前一个坐标的均方
            msd = sum((a - b) ** 2 for a, b in zip((x, y), self.window[name][-1])) / 2.0
            # print(name, msd)

            if msd > threshold:
                # 如果均方差超过阈值，可能是异常值，不将其添加到数据中
                return

        # 将坐标数据添加到数据字典和滑动窗口中。
        self.data[name].append((x, y))
        # guess_list[name] = False

        self.window[name].append((x, y))
        self.last_update[name] = time.time()  # 更新最后更新时间

    # 过滤计算滑动窗口平均值
    def filter_data(self, name):
        if name not in self.data:
            return None

        if len(self.window[name]) < self.window_size:
            return None  # 不足以进行滤波

        # 计算滑动窗口内的坐标平均值
        x_avg = sum(coord[0] for coord in self.window[name]) / self.window_size
        y_avg = sum(coord[1] for coord in self.window[name]) / self.window_size

        return x_avg, y_avg

    # 获取所有机器人坐标
    def get_all_data(self):
        filtered_d = {}
        for name in self.data:
            # 超过max_inactive_time没识别到机器人将会清空缓冲区，并进行盲区预测
            if time.time() - self.last_update[name] > self.max_inactive_time:
                self.data[name].clear()
                self.window[name].clear()
                # guess_list[name] = True
            # 识别到机器人，不进行盲区预测
            else:
                # guess_list[name] = False
                filtered_d[name] = self.filter_data(name)
        # 返回所有当前识别到的机器人及其坐标的均值
        return filtered_d

# 创建机器人坐标滤波器
filter = Filter(window_size=3, max_inactive_time=2)

# 加载模型，实例化机器人检测器和装甲板检测器
detector = YOLOv5Detector(car_engine_path, data=car_yaml_path,conf_thres=0.2, iou_thres=0.2, max_det=14, ui=True)
detector_next = YOLOv5Detector(armor_engine_path,data=armor_yaml_path, conf_thres=0.50, iou_thres=0.2,max_det=1,ui=True)


# ROS相关变量
camera_image = None
bridge = CvBridge()

def image_callback(msg):
    global camera_image
    try:
        camera_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)



#ROS
serial_pub = rospy.Publisher("robot_points_up",Serial_Send_Up,queue_size=1)
rospy.Subscriber('/camera/image1', Image, image_callback,queue_size=1)

try:
    print("线程启动中",flush=True)
    thread = threading.Thread(target=ser_send, args=(serial_pub,))
    thread.start()
    print("线程启动成功",flush=True)
except Exception as e:
    print(f"线程启动失败: {e}",flush=True)

while camera_image is None:
    print("等待图像。。。")
    time.sleep(0.5)

# 获取相机图像的画幅，限制点不超限
img0 = camera_image.copy()
img_y = img0.shape[0]
img_x = img0.shape[1]
print(img0.shape)

while True:
    # 刷新裁判系统信息UI图像
    map = map_backup.copy()
    det_time = 0
    img0 = camera_image.copy()
    ts = time.time()
    # start = time.perf_counter()
    # 第一层神经网络识别
    result0 = detector.predict(img0)
    det_time += 1
    # end = time.perf_counter()
    # diff = end - start
    # print("Time Diff: ", diff, "s")
    for detection in result0:
        cls, xywh, conf = detection
        if cls == 'car':
            left, top, w, h = xywh
            left, top, w, h = int(left), int(top), int(w), int(h)
            # 存储第一次检测结果和区域
            # ROI出机器人区域
            cropped = camera_image[top:top + h, left:left + w]
            cropped_img = np.ascontiguousarray(cropped)
            # 第二层神经网络识别
            result_n = detector_next.predict(cropped_img)
            det_time += 1
            if result_n:
                # 叠加第二次检测结果到原图的对应位置
                img0[top:top + h, left:left + w] = cropped_img

                for detection1 in result_n:
                    cls, xywh, conf = detection1
                    if cls:  # 所有装甲板都处理，可选择屏蔽一些:
                        x, y, w, h = xywh
                        x = x + left
                        y = y + top

                        # t1 = time.time()
                        # 原图中装甲板的中心下沿作为待仿射变化的点
                        # yolo11会把变量存在gpu里
                        # x_cpu = x.detach().cpu().numpy()  # 如果 x 是张量
                        # w_cpu = w.detach().cpu().numpy()
                        # y_cpu = y.detach().cpu().numpy()
                        # h_cpu = h.detach().cpu().numpy()

                        x_cpu = x
                        w_cpu = w
                        y_cpu = y
                        h_cpu = h

                        camera_point = np.array([[[min(x_cpu + 0.5 * w_cpu, img_x),min(y_cpu + 1.5 * h_cpu, img_y)]]])
                        camera_point = camera_point.astype(np.float32)
                        M_ground = M_ground.astype(np.float32)
                        # 低到高依次仿射变化
                        # 先套用地面层仿射变化矩阵
                        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_ground)
                        # 限制转换后的点在地图范围内
                        x_c = max(int(mapped_point[0][0][0]), 0)
                        y_c = max(int(mapped_point[0][0][1]), 0)
                        x_c = min(x_c, width)
                        y_c = min(y_c, height)
                        color = mask_image[y_c, x_c]  # 通过掩码图像，获取地面层的颜色：黑（0，0，0）

                        if color[0] == color[1] == color[2] == 0:
                            X_M = x_c
                            Y_M = y_c
                            # Z_M = 0
                            filter.add_data(cls, X_M, Y_M)
                        else:
                            # 不满足则继续套用R型高地层仿射变换矩阵
                            mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_r)
                            # 限制转换后的点在地图范围内
                            x_c = max(int(mapped_point[0][0][0]), 0)
                            y_c = max(int(mapped_point[0][0][1]), 0)
                            x_c = min(x_c, width)
                            y_c = min(y_c, height)
                            color = mask_image[y_c, x_c]  # 通过掩码图像，获取R型高地层的颜色：绿（0，255，0）
                            if color[1] > color[2] and color[1] > color[0]:
                                X_M = x_c
                                Y_M = y_c
                                # Z_M = 400
                                filter.add_data(cls, X_M, Y_M)
                            else:
                                # 不满足则继续套用环形高地层仿射变换矩阵
                                mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), M_height_g)
                                # 限制转换后的点在地图范围内
                                x_c = max(int(mapped_point[0][0][0]), 0)
                                y_c = max(int(mapped_point[0][0][1]), 0)
                                x_c = min(x_c, width)
                                y_c = min(y_c, height)
                                color = mask_image[y_c, x_c]  # 通过掩码图像，获取环型高地层的颜色：蓝（255，0，0）
                                if color[0] > color[2] and color[0] > color[1]:
                                    X_M = x_c
                                    Y_M = y_c
                                    # Z_M = 600
                                    filter.add_data(cls, X_M, Y_M)
                        
                        hide = hide_mask[y_c, x_c]
                        if hide[0] == 255 and hide[1] != 255:
                            guess_index[cls] = 1
                        if hide[1] == 255 and hide[2] != 255:
                            guess_index[cls] = 2
                        if hide[2] == 255 and hide[1] != 255:
                            guess_index[cls] = 3
                        else:
                            guess_index[cls] = 0

    if(True):
        # 获取所有识别到的机器人坐标
        all_filter_data = filter.get_all_data()
        if all_filter_data:  # 检查字典是否为空
            for name, xyxy in all_filter_data.items():
                if xyxy is not None:
                    if name[0] == "R":
                        color_m = (0, 0, 255)
                    else:
                        color_m = (255, 0, 0)
                    if state == 'R':
                        filtered_xyz = (2800 - xyxy[1], xyxy[0])  # 缩放坐标到地图图像
                    else:
                        filtered_xyz = (xyxy[1], 1500 - xyxy[0])  # 缩放坐标到地图图像
                    # 只绘制敌方阵营的机器人（这里不会绘制盲区预测的机器人）
                    # if name[0] != state:
                    cv2.circle(map, (int(filtered_xyz[0]), int(filtered_xyz[1])), 20, color_m, -1)  # 绘制圆
                    cv2.putText(map, str(name),
                                (int(filtered_xyz[0]) - 5, int(filtered_xyz[1]) + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 5)
                    ser_x = int(filtered_xyz[0]) * 10 / 1000
                    ser_y = int(1500 - filtered_xyz[1]) * 10 / 1000
                    cv2.putText(map, "(" + str(ser_x) + "," + str(ser_y) + ")",
                                (int(filtered_xyz[0]) - 100, int(filtered_xyz[1]) + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)
        map_show = cv2.resize(map, (1200, 640))
        cv2.imshow('map2', map_show)


    te = time.time()
    t_p = te - ts
    rospy.loginfo("fps2:%f",1 / t_p)  # 打印帧率
    # 绘制UI

    img0 = cv2.resize(img0, (1300, 900))
    cv2.imshow('img2', img0)
    
    key = cv2.waitKey(1)

