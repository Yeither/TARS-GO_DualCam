# TARS-GO_DualCam
吉大25赛季雷达站
=======
## 感谢开源
- [厦门理工学院PFA战队单目相机雷达站算法开源](https://github.com/CarryzhangZKY/pfa_vision_radar "厦门理工学院PFA战队单目相机雷达站算法开源")
- [厦理单目相机雷达站（吉大改）](https://github.com/GnehSizum/Vision_Radar "穆哥tql")

## 算法结构

## 环境配置
- 1.硬件部分包括NVIDIA GeForce RTX 3060 Laptop GPU (6GB)和AMD Ryzen 7 5800H with Radeon Graphics
- 2.系统为Ubuntu20.04，CUDA版本为11.8，Python版本为3.10，使用了Cudnn9.1.0.70和TensorRT 10.8.0.43进行加速
- 3.鱼香ROS配置ros
  ```bash
  wget http://fishros.com/install -O fishros && . fishros
- 4.相机驱动以海康为例 [驱动下载](https://open.hikvision.com/download/5cda567cf47ae80dd41a54b3?type=10 "hik")
- 5.python需要的环境
  ```bash
  pip install -r requirements.txt
- 6.需要去[PyQT官网](https://pypi.org/project/PyQt5/ "PyQT")下载PyQT，使用pip下载版本不够

## 使用方法（launch文件解读）

## 优化意见
