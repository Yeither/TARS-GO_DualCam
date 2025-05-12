import numpy as np
import cv2
import concurrent.futures


class Stitcher:
    # 拼接函数
    def stitch(self, images, direction='vertical', ratio=0.75, reprojThresh=4.0, showMatches=False):
        # 获取输入图片
        (imageB, imageA) = images
        # 并行进行特征检测和描述
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futureA = executor.submit(self.detectAndDescribe, imageA)
            futureB = executor.submit(self.detectAndDescribe, imageB)
            (kpsA, featuresA) = futureA.result()
            (kpsB, featuresB) = futureB.result()

        M = self.matchKeypoints(kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh)
        # 如果返回结果为空，没有匹配成功的特征点，退出算法
        if M is None:
            return None
        # 否则，提取匹配结果
        # H是3x3视角变换矩阵
        (matches, H, status) = M

        # 检查 H 是否为 None
        if H is None:
            print("Error: Homography matrix is None.")
            return None

        # 检查 H 的数据类型
        if H.dtype != np.float32 and H.dtype != np.float64:
            H = H.astype(np.float32)

        # 检查 H 的形状
        if H.shape != (3, 3):
            print("Error: Homography matrix is not a 3x3 matrix.")
            return None

        if direction == 'horizontal':
            # 左右拼接
            result = cv2.warpPerspective(imageA, H, (imageA.shape[1] + imageB.shape[1], imageA.shape[0]))
            result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
        elif direction == 'vertical':
            # 上下拼接
            result = cv2.warpPerspective(imageA, H, (imageA.shape[1], imageA.shape[0] + imageB.shape[0]))
            result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
        else:
            raise ValueError("Direction must be either 'horizontal' or 'vertical'")

        # 检测是否需要显示图片匹配
        if showMatches:
            # 生成匹配图片
            vis = self.drawMatches(imageA, imageB, kpsA, kpsB, matches, status)
            # 返回结果
            return (result, vis)
        # 返回匹配结果
        return result

    def detectAndDescribe(self, image):
        # 将彩色图片转换成灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 调整SIFT参数以减少计算量
        descriptor = cv2.SIFT_create(nfeatures=500)
        (kps, features) = descriptor.detectAndCompute(image, None)
        kps = np.float32([kp.pt for kp in kps])
        return (kps, features)

    def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh):
        # 建立暴力匹配器
        matcher = cv2.BFMatcher()
        # 使用KNN检测来自A、B图的SIFT特征匹配对，K=2
        rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
        matches = []
        for m in rawMatches:
            # 当最近距离跟次近距离的比值小于ratio值时，保留此匹配对
            if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                # 存储两个点在featuresA, featuresB中的索引值
                matches.append((m[0].trainIdx, m[0].queryIdx))
        # 当筛选后的匹配对大于4时，计算视角变换矩阵
        if len(matches) > 4:
            # 获取匹配对的点坐标
            ptsA = np.float32([kpsA[i] for (_, i) in matches])
            ptsB = np.float32([kpsB[i] for (i, _) in matches])
            # 调整RANSAC参数以减少迭代次数
            (H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC, reprojThresh, maxIters=200)
            # 返回结果
            return (matches, H, status)
        # 如果匹配对小于4时，返回None
        return None

    def drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
        # 初始化可视化图片，将A、B图左右连接到一起
        (hA, wA) = imageA.shape[:2]
        (hB, wB) = imageB.shape[:2]
        vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
        vis[0:hA, 0:wA] = imageA
        vis[0:hB, wA:] = imageB

        # 联合遍历，画出匹配对
        for ((trainIdx, queryIdx), s) in zip(matches, status):
            # 当点对匹配成功时，画到可视化图上
            if s == 1:
                # 画出匹配对
                ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
                ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
                cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

        # 返回可视化结果
        return vis
    
    def fast_attacher(self,image1,image2,alpha):
        # 确保两张图片的宽度一致
        if image1.shape[1] != image2.shape[1]:
            image2 = cv2.resize(image2, (image1.shape[1], image2.shape[0]))

        # 计算第一张图片要截取的高度
        height_to_crop = int(image1.shape[0] * alpha)

        # 截取第一张图片的上半部分
        cropped_image1 = image1[:height_to_crop, :]

        # 拼接图片
        result_image = cv2.vconcat([cropped_image1, image2])

        return result_image
    def ORB_stitch_images(self, img1, img2):
        # 初始化ORB检测器
        orb = cv2.ORB_create()

        # 检测关键点和描述符
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        # 检查描述符是否为空
        if des1 is None or des2 is None:
            print("描述符为空，无法进行匹配")
            return None

        # 检查描述符的数据类型和列数
        if des1.dtype != des2.dtype or des1.shape[1] != des2.shape[1]:
            print(f"描述符数据类型或列数不匹配: des1 dtype={des1.dtype}, des2 dtype={des2.dtype}, des1 cols={des1.shape[1]}, des2 cols={des2.shape[1]}")
            return None

        # 使用BFMatcher进行特征匹配
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)

        # 按照距离排序匹配点
        matches = sorted(matches, key=lambda x: x.distance)

        # 提取匹配点坐标
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # 使用RANSAC算法计算单应矩阵
        # 检查匹配点数量是否至少为4个
        if len(matches) < 4:
            print("匹配点数量不足，无法计算单应性矩阵")
            # 可以选择返回原始图像或者进行其他处理
            return img1
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        # 应用单应矩阵进行图像拼接
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        # 计算拼接后图像的尺寸
        pts1 = np.float32([[0, 0], [0, h1], [w1, h1], [w1, 0]]).reshape(-1, 1, 2)
        pts2 = np.float32([[0, 0], [0, h2], [w2, h2], [w2, 0]]).reshape(-1, 1, 2)
        pts1_transformed = cv2.perspectiveTransform(pts1, M)
        pts = np.concatenate((pts2, pts1_transformed), axis=0)
        [xmin, ymin] = np.int32(pts.min(axis=0).ravel() - 0.5)
        [xmax, ymax] = np.int32(pts.max(axis=0).ravel() + 0.5)
        t = [-xmin, -ymin]
        Ht = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]])  # 平移矩阵

        # 进行透视变换并拼接图像
        result = cv2.warpPerspective(img1, Ht.dot(M), (xmax - xmin, ymax - ymin))

        # 确保拼接区域的形状匹配
        h_result, w_result = result.shape[:2]
        h_offset = max(0, t[1])
        w_offset = max(0, t[0])
        h_overlap = min(h2, h_result - h_offset)
        w_overlap = min(w2, w_result - w_offset)

        result[h_offset:h_offset + h_overlap, w_offset:w_offset + w_overlap] = img2[:h_overlap, :w_overlap]

        return result
