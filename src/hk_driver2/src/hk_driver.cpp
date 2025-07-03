#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <mutex>
#include <thread>
#include <iostream>

using namespace std;

bool g_bExit = false;  // 线程结束标记
#define Max_Count 5       // 队列 缓冲区个数； 根据实际情况调节
uint64_t m_nImageSize = 0;    // 图像大小

// 自定义保存的图像信息的结构体
typedef struct _stImageNode_
{
    unsigned char* pData;
    uint64_t nFrameLen;
    
    unsigned int nWidth;
    unsigned int nHeight;
    unsigned int nFrameNum;
}stImageNode;

class ArrayQueue
{
public:
    ArrayQueue()
    {
        this->size = 0;
        this->start = 0;
        this->end = 0;
        this->Queue = NULL;
        this->Qlen = 0;
    }

    ~ArrayQueue()
    {
        g_mutex.lock();

        for (int i = 0; i< Qlen; i++)
        {
            Queue[i].nFrameNum = 0;
            Queue[i].nHeight = 0;
            Queue[i].nWidth = 0;
            Queue[i].nFrameLen = 0;  
            if(Queue[i].pData != NULL)
            {
                free(Queue[i].pData);
                Queue[i].pData = NULL;
            }                    
        }

        delete []Queue;
        Queue = NULL;

        size = 0;
        start = 0;
        end = 0;
        g_mutex.unlock();
    }

    int Init(int nBufCount, uint64_t DefaultImagelen)
    {
        int nRet = 0;

        this->Queue = new (std::nothrow)stImageNode[nBufCount];
        if (this->Queue == NULL)
        {
            return MV_E_RESOURCE;
        }
        this->Qlen = nBufCount;

        for (int i = 0; i< nBufCount; i++)
        {
            Queue[i].nFrameNum = 0;
            Queue[i].nHeight = 0;
            Queue[i].nWidth = 0;
            Queue[i].nFrameLen = 0;          
            Queue[i].pData = (unsigned char*)malloc(DefaultImagelen);
            if(NULL ==  Queue[i].pData)
            {
                return MV_E_RESOURCE;
            }
        }

        return 0;
    }

    int push(int nFrameNum, int nHeight, int nWidth, unsigned char *pData, uint64_t nFrameLen)
    {
        g_mutex.lock();
        
        if (size==Qlen)
        {
            g_mutex.unlock();
            return MV_E_BUFOVER;
        }

        size++;
        Queue[end].nFrameNum = nFrameNum;
        Queue[end].nHeight = nHeight;
        Queue[end].nWidth = nWidth;
        Queue[end].nFrameLen = nFrameLen;

        if (NULL !=  Queue[end].pData  && NULL != pData)
        {
            memcpy(Queue[end].pData, pData, nFrameLen);
        }

        end = end == Qlen - 1 ? 0 : end + 1;
        g_mutex.unlock();
        return 0;
    }

    int poll(int &nFrameNum, int &nHeight, int &nWidth, unsigned char *pData, int &nFrameLen)
    {
        g_mutex.lock();

        if (size == 0)
        {
            g_mutex.unlock();
            return  MV_E_NODATA;
        }

        nFrameNum =Queue[start].nFrameNum;
        nHeight =Queue[start].nHeight;
        nWidth =Queue[start].nWidth;
        nFrameLen =Queue[start].nFrameLen;

        if (NULL !=  pData && NULL != Queue[start].pData)
        {
            memcpy( pData,Queue[start].pData, nFrameLen);
        }

        size--;
        start = start == Qlen - 1 ? 0 : start + 1;

        g_mutex.unlock();
        return 0;
    }

private:
    stImageNode *Queue;
    int size;
    int start;
    int end;
    int Qlen;

    std::mutex g_mutex; // 互斥锁
};

ArrayQueue * m_queue = NULL;  // 线程通信队列

ros::Publisher image_pub;  // ROS 图像发布器

// 图像回调函数
void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo == NULL || pData == NULL)
    {
        ROS_ERROR("Invalid parameters passed to ImageCallback");
        return;
    }

    // 将图像数据转换为 OpenCV 格式（Bayer RG8）
    cv::Mat bayerImage(pFrameInfo->nExtendHeight, pFrameInfo->nExtendWidth, CV_8UC1, pData);

    // 将 Bayer RG8 转换为 RGB8
    cv::Mat rgbImage;
    cv::cvtColor(bayerImage, rgbImage, cv::COLOR_BayerRG2RGB);

    // 将 OpenCV 图像转换为 ROS 图像消息
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImage).toImageMsg();

    // 发布图像
    image_pub.publish(msg);
}

// 等待用户输入 Enter 键来结束取流或结束程序
void PressEnterToExit(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
    ROS_INFO("Press enter to exit.");
    while (getchar() != '\n');
    g_bExit = true;
    sleep(1);
}

// 异常回调函数：设备断开时自动重新连接
void __stdcall ReconnectDevice(unsigned int nMsgType, void* pUser)
{
    if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT) // 设备断开
    {
        ROS_WARN("Device disconnected. Attempting to reconnect...");

        void* handle = (void*)pUser;
        int nRet = MV_OK;
        
        // 清理现有的设备连接
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);

        do
        {
            // 尝试重新打开设备
            ROS_INFO("Reconnecting to the device...");

            MV_CC_DEVICE_INFO_LIST stDeviceList;
            memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
            if (nRet != MV_OK || stDeviceList.nDeviceNum == 0)
            {
                ROS_ERROR("Device enumeration failed, retrying...");
                sleep(1);
                continue;
            }

            // 重新选择设备并创建连接
            unsigned int nIndex = 0;
            nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
            if (nRet != MV_OK)
            {
                ROS_ERROR("Failed to create device handle.");
                sleep(1);
                continue;
            }

            nRet = MV_CC_OpenDevice(handle);
            if (nRet != MV_OK)
            {
                ROS_ERROR("Failed to open device.");
                sleep(1);
                continue;
            }

            // 重新设置图像回调
            nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, NULL);
            if (nRet != MV_OK)
            {
                ROS_ERROR("Failed to register image callback.");
                sleep(1);
                continue;
            }

            // 重新开始抓取图像
            nRet = MV_CC_StartGrabbing(handle);
            if (nRet != MV_OK)
            {
                ROS_ERROR("Failed to start grabbing.");
                sleep(1);
                continue;
            }

            ROS_INFO("Reconnection successful.");
            break; // 连接成功，退出循环
        } while (g_bExit == false);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_publisher1");
    ros::NodeHandle nh;

    // 创建 ROS 图像发布器，发布到 "/camera/image" 话题
    image_pub = nh.advertise<sensor_msgs::Image>("camera/image1", 1);

    int nRet = MV_OK;
    void* handle = NULL;

    do 
    {
        // 1. 初始化 SDK
        nRet = MV_CC_Initialize();
        if (MV_OK != nRet)
        {
            ROS_ERROR("Initialize SDK fail! nRet [0x%x]", nRet);
            break;
        }

        // 2. 枚举设备
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_EnumDevices fail! nRet [%x]", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                ROS_INFO("[device %d]:", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
            }  
        } 
        else
        {
            ROS_ERROR("Find No Devices!");
            break;
        }

        // 3. 初始化设备
        unsigned int nIndex = 1;
        if (nIndex >= stDeviceList.nDeviceNum)
        {
            ROS_ERROR("Input error!");
            break;
        }

        // 4. 创建设备句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_CreateHandle fail! nRet [%x]", nRet);
            break;
        }

        // 5. 打开设备
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_OpenDevice fail! nRet [%x]", nRet);
            break;
        }

        // 6. 设置曝光时间
        float ExposureTimeValue = 8000.0f;  // 设置曝光时间（单位：微秒，根据需要调整）
        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", ExposureTimeValue);
        if (MV_OK != nRet)
        {
            printf("Set ExposureTime fail! nRet [0x%x]\n", nRet);
            return nRet;
        }

        // 7. 设置增益
        float GainValue = 17.9f;  // 设置增益值，根据需要调整
        nRet = MV_CC_SetFloatValue(handle, "Gain", GainValue);
        if (MV_OK != nRet)
        {
            printf("Set Gain fail! nRet [0x%x]\n", nRet);
            return nRet;
        }

        // 8. 注册异常回调函数
        nRet = MV_CC_RegisterExceptionCallBack(handle, ReconnectDevice, handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_RegisterExceptionCallBack fail! nRet [%x]", nRet);
            break;
        }

        // 9. 注册图像回调函数
        nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, NULL);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_RegisterImageCallBackEx fail! nRet [%x]", nRet);
            break;
        }

        // 10. 启动图像抓取
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]", nRet);
            break;
        }

        // 11. 等待用户输入来结束程序
        PressEnterToExit(); // 等待用户输入来结束

    } while (0);

    if (handle != NULL)
    {
        MV_CC_DestroyHandle(handle);
                
        MV_CC_StopGrabbing(handle); 

        MV_CC_CloseDevice(handle);
    }

    return 0;
}
