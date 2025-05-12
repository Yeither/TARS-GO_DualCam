#!/usr/bin/env python3

import rospy
from detect.msg import Serial_Send_Down
from detect2.msg import Serial_Send_Up
import sys

sys.path.insert(0, "/home/yang/double_camera/src/my_serial/scripts")
from RM_serial_py.ser_api import *
import serial
import queue
import time
from RM_serial_py.ser_api import build_send_packet, receive_packet, Radar_decision, \
    build_data_decision, build_data_radar,build_data_radar_sentry
import numpy as np
import cv2
from information_ui import draw_information_ui

down_queue = queue.Queue()
up_queue = queue.Queue()
seq_s = 0  # 假设seq_s是全局变量
global chances_flag
chances_flag = 1

# 全局变量
# 初始化战场信息UI（标记进度、双倍易伤次数、双倍易伤触发状态）
information_ui = np.zeros((500, 420, 3), dtype=np.uint8) * 255
information_ui_show = information_ui.copy()
progress_list = [-1, -1, -1, -1, -1, -1] 
double_vulnerability_chance = -1
opponent_double_vulnerability = -1
target = -1
mark_progress = {}
target_last = 0  # 上一帧的飞镖目标

state = 'R'  # 假设state是全局变量


def convert_to_uint16(data_list):
    uint16_list = []
    for point in data_list:
        # 遍历每个点的坐标
        uint16_point = []
        for coord in point:
            # 对浮点数进行取整
            int_coord = int(coord)
            # 确保取值在 uint16 范围内
            uint16_coord = max(0, min(65535, int_coord))
            uint16_point.append(uint16_coord)
        uint16_list.append(tuple(uint16_point))
    return uint16_list

# 裁判系统串口接收函数，在主循环中调用
def ser_receive(ser):
    global progress_list  # 标记进度列表
    global double_vulnerability_chance  # 拥有双倍易伤次数
    global opponent_double_vulnerability  # 双倍易伤触发状态
    global target  # 飞镖当前目标
    progress_cmd_id = [0x02, 0x0C]  # 任意想要接收数据的命令码，这里是雷达标记进度的命令码0x020E
    vulnerability_cmd_id = [0x02, 0x0E]  # 双倍易伤次数和触发状态
    target_cmd_id = [0x01, 0x05]  # 飞镖目标
    buffer = b''  # 初始化缓冲区

    # 从串口读取数据
    received_data = ser.read_all()  # 读取一秒内收到的所有串口数据
    # 将读取到的数据添加到缓冲区中
    buffer += received_data

    # 查找帧头（SOF）的位置
    sof_index = buffer.find(b'\xA5')

    while sof_index != -1:
        # 如果找到帧头，尝试解析数据包
        if len(buffer) >= sof_index + 5:  # 至少需要5字节才能解析帧头
            # 从帧头开始解析数据包
            packet_data = buffer[sof_index:]

            # 查找下一个帧头的位置
            next_sof_index = packet_data.find(b'\xA5', 1)

            if next_sof_index != -1:
                # 如果找到下一个帧头，说明当前帧头到下一个帧头之间是一个完整的数据包
                packet_data = packet_data[:next_sof_index]
                # print(packet_data)
            else:
                # 如果没找到下一个帧头，说明当前帧头到末尾不是一个完整的数据包
                break

            # 解析数据包
            progress_result = receive_packet(packet_data, progress_cmd_id,
                                                info=False)  # 解析单个数据包，cmd_id为0x020E,不输出日志    020C
            vulnerability_result = receive_packet(packet_data, vulnerability_cmd_id, info=False)     # O2OE
            target_result = receive_packet(packet_data, target_cmd_id, info=False)                  # 0105
            # print("progress_result:  ",progress_result)
            # print("vulnerability_result:  ",vulnerability_result)
            # print("target_result: ",target_result)
            # 更新裁判系统数据，标记进度、易伤、飞镖目标
            if progress_result is not None:
                received_cmd_id1, received_data1, received_seq1 = progress_result
                # print("received_data1:    ",received_data1)
                num = int.from_bytes(received_data1, byteorder='big')
                progress_list = [(num >> (7 - i)) & 1 for i in range(8)]
                # print("progress_list: ", progress_list, "type: ", type(progress_list[0]))
                
                if state == 'R':
                    mark_progress['B1'] = progress_list[0]
                    mark_progress['B2'] = progress_list[1]
                    mark_progress['B3'] = progress_list[2]
                    mark_progress['B4'] = progress_list[3]
                    mark_progress['B7'] = progress_list[5]
                else:
                    mark_progress['R1'] = progress_list[0]
                    mark_progress['R2'] = progress_list[1]
                    mark_progress['R3'] = progress_list[2]
                    mark_progress['R4'] = progress_list[3]
                    mark_progress['R7'] = progress_list[5]
            if vulnerability_result is not None:
                received_cmd_id2, received_data2, received_seq2 = vulnerability_result
                received_data2 = list(received_data2)[0]
                double_vulnerability_chance, opponent_double_vulnerability = Radar_decision(received_data2)

                
            if target_result is not None:
                received_cmd_id3, received_data3, received_seq3 = target_result
                target = (list(received_data3)[1] & 0b1100000) >> 5

            # 创建消息并发布
            # print("progress_list!!!:"+ str(progress_list))
            # rospy.loginfo("progress_list!!!:"+ str(progress_list))
            
            # 从缓冲区中移除已解析的数据包
            buffer = buffer[sof_index + len(packet_data):]

            # 继续寻找下一个帧头的位置
            sof_index = buffer.find(b'\xA5')

        else:
            # 缓冲区中的数据不足以解析帧头，继续读取串口数据
            break


def callback_down(data):
    try:
        # rospy.loginfo("Received data from robot_points_down:")
        # rospy.loginfo("position_1_down: %s", data.position_1_down)
        # rospy.loginfo("position_2_down: %s", data.position_2_down)
        # rospy.loginfo("position_3_down: %s", data.position_3_down)
        # rospy.loginfo("position_4_down: %s", data.position_4_down)
        # rospy.loginfo("position_7_down: %s", data.position_7_down)
        down_queue.put(data)
    except AttributeError as e:
        rospy.logerr(f"Error processing robot_points_down message: {e}")


def callback_up(data):
    try:
        # rospy.loginfo("Received data from robot_points_up:")
        # rospy.loginfo("position_1_up: %s", data.position_1_up)
        # rospy.loginfo("position_2_up: %s", data.position_2_up)
        # rospy.loginfo("position_3_up: %s", data.position_3_up)
        # rospy.loginfo("position_4_up: %s", data.position_4_up)
        # rospy.loginfo("position_7_up: %s", data.position_7_up)
        up_queue.put(data)
    except AttributeError as e:
        rospy.logerr(f"Error processing robot_points_up message: {e}")


def listener():
    global target  # 飞镖当前目标
    global target_last  # 飞镖当前目标
    global information_ui_show

    rospy.init_node('data_receiver', anonymous=True)
    seq = False
    try:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        print("初始化串口成功，使用 /dev/ttyACM1")
    except serial.SerialException:
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            print("初始化串口成功，使用 /dev/ttyACM0")
        except serial.SerialException:
            try:
                ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
                print("初始化串口成功，使用 /dev/ttyUSB0")
            except serial.SerialException as e:
                rospy.logerr(f"Failed to open serial port: {e}")
                return

    rospy.Subscriber("robot_points_down", Serial_Send_Down, callback_down, queue_size=1)
    rospy.Subscriber("robot_points_up", Serial_Send_Up, callback_up, queue_size=1)


    rate = rospy.Rate(30)  # 30Hz
    while not rospy.is_shutdown():
        global information_ui
        global information_ui_show
        information_ui_show = information_ui.copy()
        # 先尝试接收数据
        ser_receive(ser)

        if (not down_queue.empty()) and (not up_queue.empty()):
            data = up_queue.get()
            down_data = down_queue.get()

            # 先把上面识别的装进去要是上面是猜的就拿下面相机的数据覆盖
            data_list = [data.position_1_up, data.position_2_up, data.position_3_up, data.position_4_up, data.position_7_up]
            down_data_list = [down_data.position_1_down, down_data.position_2_down, down_data.position_3_down, down_data.position_4_down, down_data.position_7_down]
            data_guess_list = data.if_guess
            down_data_guess_list = down_data.if_guess
            send_sentry = data_list

            for num in range(0,4):
                if data_guess_list[num]:
                    data_list[num] = down_data_list[num]
                    if down_data_guess_list[num]:
                        send_sentry[num] = (0,0)
                    else:
                        send_sentry[num] = down_data_list[num]
                else:
                    send_sentry[num] = data_list[num]

            #  转16位发送
            uint16_data_list=convert_to_uint16(data_list)
            # print("uint16_data_list:  ",uint16_data_list)
            ser_data = build_data_radar(uint16_data_list)
            # print("ser_data:  ",ser_data)
            packet, seq = build_send_packet(ser_data, seq, [0x03, 0x05])
            # print("packet:  ",packet)
            ser.write(packet)

            uint16_send_sentry = convert_to_uint16(send_sentry)
            # print("uint16_send_sentry: ",uint16_send_sentry)
            ser_send_sentry_data = build_data_radar_sentry(state,uint16_send_sentry)
            # print("ser_send_sentry_data: ",ser_send_sentry_data)
            sentry_packet, seq = build_send_packet(ser_send_sentry_data, seq, [0x03, 0x01])
            # print("sentry_packet:  ",sentry_packet)
            ser.write(sentry_packet)
            # time.sleep()

            # 有双倍易伤机会，并且当前没有在双倍易伤
            # 判断飞镖的目标是否切换，切换则尝试发动双倍易伤
            
            if target != target_last and target != 0:
                target_last = target
                # 有双倍易伤机会，并且当前没有在双倍易伤
                if double_vulnerability_chance > 0 and opponent_double_vulnerability == 0:
                    time_e = time.time()
                    # 发送时间间隔为10秒
                    if time_e - time_s > 10:
                        print("请求双倍触发")
                        data = build_data_decision(chances_flag, state)
                        packet, seq = build_send_packet(data, seq, [0x03, 0x01])
                        print(packet.hex(),chances_flag,state)
                        ser.write(packet)
                        print("请求成功", chances_flag)
                        # 更新标志位
                        chances_flag += 1
                        if chances_flag >= 3:
                            chances_flag = 1

                        time_s = time.time()

                        
        _ = draw_information_ui(progress_list, state, information_ui_show)
        cv2.putText(information_ui_show, "vulnerability_chances: " + str(double_vulnerability_chance),
                (10, 350),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(information_ui_show, "vulnerability_Triggering: " + str(opponent_double_vulnerability),
                (10, 400),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow('information_ui', information_ui_show)
        cv2.waitKey(1)

        rate.sleep()
        

if __name__ == '__main__':
    listener()
