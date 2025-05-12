import random
import yaml
from ultralytics import YOLO
from ultralytics.utils.torch_utils import select_device
# from ultralytics.utils.ops import non_max_suppression, xyxy2xywh, scale_boxes
from yyy_utils import py_cpu_nms
import torch
from ultralytics.utils.ops import non_max_suppression, xyxy2xywh
from ultralytics.utils.plotting import Annotator
import numpy as np
from ultralytics.nn.autobackend import AutoBackend


class YOLO11Detector:
    def __init__(self,weights_path,batch_size,conf_thres,iou_thres = 0.2,classes=None,agnostic_nms=False,max_det=10,data="yaml/armor.yaml",ui=True,half=True, dnn=False,device=''):
        # self.model = YOLO(weights_path)
        # 加载模型
        self.device = select_device(device)
        # print("！！！！weights_path: ",weights_path)
        self.model = YOLO(weights_path)

        self.batch_size = batch_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.max_det = max_det
        self.ui = ui

        self.names = self.model.names
        # 在初始化时，确保 self.colors 是一个元组列表，而不是列表的列表
        self.colors = [tuple([random.randint(0, 255) for _ in range(3)]) for _ in self.names]
        self.data = data
        # 加载 YAML 文件内容
        with open(data, 'r') as f:
            data_ = yaml.safe_load(f)  # 读取并解析 YAML 文件

        self.data = data_
        # 获取 'names' 的种类数
        self.num_classes = len(self.data["names"])  # 获取类别数

    def predict(self, img, imgsz=512):
        # 获取模型预测结果
        prediction = self.model.predict(img,agnostic_nms=True,conf = self.conf_thres,iou = self.iou_thres,half = True,imgsz=imgsz)[0]


        # 整合框坐标、置信度和类别信息
        detections = prediction.boxes.data

        # 处理 NMS 后的框并绘制
        final_detections = []
        for det in detections:
            # print("det: ",det)
            if len(det):
                # 赋值给 *xyxy, conf, cls
                xyxy = det[:4]  # 前四个值是边界框的坐标 (x1, y1, x2, y2)
                conf = det[4]  # 第五个值是置信度
                cls = det[5]  # 第六个值是类别索引
                # print(f"Coordinates: {xyxy}, Confidence: {conf}, Class: {cls}")
                label = f'{self.names[int(cls)]} {conf:.2f}'
                if self.ui:
                    annotator = Annotator(np.ascontiguousarray(img), line_width=3, example=str(self.names))
                    annotator.box_label(xyxy, label, color=self.colors[int(cls)])
                final_detections.append((self.names[int(cls)], xyxy, float(conf)))

        return final_detections
