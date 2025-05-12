### /home/yang/double_camera/src/detect/scripts/images/map.jpg
import cv2
import numpy as np

def mouse_callback(event, x, y, flags, param):
    img_copy = param['img_copy']
    original_img = param['original_img'].copy()  # 使用副本避免修改原图
    original_height = param['original_height']
    
    if event == cv2.EVENT_MOUSEMOVE:
        # 鼠标移动时显示坐标（左下角为原点）
        original_x = x * 2
        original_y = y * 2
        print(f"鼠标悬停处坐标 (左下角为原点): ({original_x}, {original_height - original_y})")
    
    elif event == cv2.EVENT_LBUTTONDOWN:
        # 鼠标左键点击时绘制同心圆
        # 清除之前的圆，创建原图的新副本
        original_img = param['original_img'].copy()
        
        # 将显示图像中的坐标扩大2倍，得到原始图像中的坐标
        center_x = x * 2
        center_y = y * 2
        
        # 转换为OpenCV坐标系（左上角为原点）
        cv_center_y = center_y
        
        # 在原始图像上绘制两个同心圆（半径40和80像素）
        cv2.circle(original_img, (center_x, cv_center_y), 40, (0, 255, 0), 2)  # 绿色小圆
        cv2.circle(original_img, (center_x, cv_center_y), 80, (0, 0, 255), 2)  # 红色大圆
        
        # 重新缩放图像用于显示
        img_small = cv2.resize(original_img, (original_img.shape[1]//2, original_img.shape[0]//2))
        img_copy[:] = img_small[:]  # 确保尺寸匹配
        
        print(f"在坐标 ({center_x}, {original_height - center_y}) 处绘制了圆 (左下角为原点)")

if __name__ == "__main__":
    image_path = "/home/yang/double_camera/src/detect/scripts/images/map.jpg"  # 请替换为你的图片路径
    original_img = cv2.imread(image_path)
    original_height, original_width, _ = original_img.shape
    
    # 创建用于显示的图像副本（初始为缩小后的原图）
    img_small = cv2.resize(original_img, (original_width//2, original_height//2))
    img_copy = img_small.copy()
    
    cv2.namedWindow('Image')
    params = {
        'img_copy': img_copy,
        'original_img': original_img.copy(),  # 保存原图副本
        'original_height': original_height
    }
    cv2.setMouseCallback('Image', mouse_callback, params)

    while True:
        cv2.imshow('Image', img_copy)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

