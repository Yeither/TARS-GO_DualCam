import cv2
import numpy as np

# 全局配置
image_path = "/home/yang/double_camera/cal/images/map_red.jpg"  # 图片路径
display_scale = 0.3  # 显示缩放比例
colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # 红、绿、蓝
current_color_idx = 0
clicked_points = []  # 存储格式: [(x, y, color_idx), ...]


def on_mouse_click(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        # 计算原始坐标（考虑缩放）
        orig_x = int(x / display_scale)
        orig_y = int(y / display_scale)

        # 转换为右下角原点坐标
        height, width = param['original_shape']
        converted_x = width - 1 - orig_x
        converted_y = height - 1 - orig_y

        # 存储信息
        clicked_points.append((orig_x, orig_y, current_color_idx))

        # 更新显示
        update_display(param['display_img'], param['original_shape'])

        # 打印坐标信息
        print(f"标号 {len(clicked_points)}: 原始像素 ({orig_x}, {orig_y}) → 右下原点 ({converted_x}, {converted_y})")


def update_display(display_img, original_shape):
    """更新显示图像（带所有标记点）"""
    # 重新缩放原始图像
    display_img[:] = cv2.resize(original_img, (0, 0), fx=display_scale, fy=display_scale)

    # 绘制所有标记点
    for i, (x, y, color_idx) in enumerate(clicked_points):
        # 转换为显示坐标
        disp_x = int(x * display_scale)
        disp_y = int(y * display_scale)

        # 绘制点和编号
        cv2.circle(display_img, (disp_x, disp_y), 2, colors[color_idx], -1)
        cv2.putText(display_img, str(i ), (disp_x + 10, disp_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[color_idx], 2)


# 加载图片
original_img = cv2.imread(image_path)
if original_img is None:
    print(f"错误：无法加载图片，请检查路径: {image_path}")
    exit()

# 初始化显示图像
display_img = cv2.resize(original_img.copy(), (0, 0), fx=display_scale, fy=display_scale)
original_shape = original_img.shape[:2]  # (height, width)

# 创建窗口
cv2.namedWindow("Image Marker")
cv2.setMouseCallback("Image Marker", on_mouse_click,
                     {'display_img': display_img, 'original_shape': original_shape})

print("操作说明:")
print("- 左键点击: 添加标记点")
print("- 空格键: 切换颜色 (红→绿→蓝)")
print("- ESC键: 保存并退出")

while True:
    cv2.imshow("Image Marker", display_img)
    key = cv2.waitKey(0)

    if key == 32:  # 空格键切换颜色
        current_color_idx = (current_color_idx + 1) % len(colors)
        print(f"当前标记颜色: {colors[current_color_idx]}")

    elif key == 27:  # ESC键保存并退出
        # 在原始图像上绘制标记
        marked_img = original_img.copy()
        for i, (x, y, color_idx) in enumerate(clicked_points):
            cv2.circle(marked_img, (x, y), 10, colors[color_idx], -1) 
            cv2.putText(marked_img, str(i + 1), (x + 15, y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, colors[color_idx], 2)

        # 保存结果
        save_path = image_path.replace(".jpg", "_marked_close.jpg")
        cv2.imwrite(save_path, marked_img)
        print(f"标记结果已保存到: {save_path}")
        break

cv2.destroyAllWindows()

# 输出所有标记点信息（右下角坐标系）
print("\n所有标记点（右下角原点坐标系）:")
for i, (x, y, color_idx) in enumerate(clicked_points):
    rx = original_shape[1] - 1 - x
    ry = original_shape[0] - 1 - y
    print(f"标号 {i + 1}: ({rx}, {ry}), 颜色 {colors[color_idx]}")
