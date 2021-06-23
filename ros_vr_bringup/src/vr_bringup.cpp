/***************************************************************************************/
// THIS FILE IS PART OF PROJECT: Immersive simulation environment
// Lib 406
//
// vr_bringup.cpp - Display image in HMD, broadcast devices' TF
//
// Created on 2021.4.8
// author:菜伟
// email:li_wei_96@163.com
/***************************************************************************************/
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <GL/glut.h>
#include <SDL2/SDL_opengl.h>
#include <GL/glu.h>

#include <openvr.h>
using namespace std;
using namespace cv;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;

vr::TrackedDevicePose_t trackedDevicePose; // 设备姿态（复用）
vr::VRControllerState_t controllerState;   // 控制器状态(复用)
vr::HmdMatrix34_t poseMatrix;              // 设备姿态（复用）
geometry_msgs::TransformStamped tf_;       // 利用标准的msg（复用）
vr::IVRSystem *m_pVRSystem;
vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

int msg_img_width = 2048;
int msg_img_height = 2048;
GLuint textureID; // 生成纹理 ID

Mat frame_l;
Mat frame_r;
ros::Publisher pub_img;
uint32_t counter = 0;

//
// 初始化世界坐标系
bool Init_world()
{
    tf_.header.frame_id = "World"; // 设置frame_ID
    tf_.child_frame_id = "vr_world";
    tf2::Matrix3x3 r_;
    r_.setValue(0, 0, 1,
                1, 0, 0,
                0, 1, 0);
    tf2::Quaternion q_;
    r_.getRotation(q_);
    tf_.transform.rotation.x = q_.getX();
    tf_.transform.rotation.y = q_.getY();
    tf_.transform.rotation.z = q_.getZ();
    tf_.transform.rotation.w = q_.getW();
    tf_.header.stamp = ros::Time::now();
    static tf2_ros::StaticTransformBroadcaster brStatic;
    brStatic.sendTransform(tf_);
    tf_.header.frame_id = "vr_world"; // 设置frame_ID
    return 1;
}

//
// 初始化GL
bool Init_GL()
{
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE); // 窗口类型（颜色|单/双缓存）
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(200, 200);
    glutCreateWindow("HMD monitor");

    glClearColor(0.2, 0.3, 0.3, 1.0);       // 背景颜色，清除缓冲区时填充的颜色
    glMatrixMode(GL_PROJECTION);            // 表示下面的操作为设置投影矩阵         参数有：GL_PROJECTION, GL_MODELVIEW, GL_TEXTURE
    glOrtho(-1.1, 1.1, -1.1, 1.1, -15, 15); // (left, right, bottom, top, near, far)
    glMatrixMode(GL_MODELVIEW);             // 设置模型视景矩阵
    gluLookAt(0, 0, 14, 0, 0, 0, 0, 1, 0);  // 摄像机的位置、规定观察点、规定摄像机的垂直朝向
    glEnable(GL_TEXTURE_2D);
    ROS_INFO_STREAM("GL runtime ");

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);                                          // 缩小过滤，
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);                                          // 放大过滤，
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);                                               // S方向
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);                                               // T方向
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, msg_img_width, msg_img_height, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL); //TODO*************************
}

//
// 将全局变量 poseMatrix 转化成 tf_。用于发布设备位姿
void setTransformStamped()
{
    tf2::Matrix3x3 R;
    R.setValue(-poseMatrix.m[0][2], -poseMatrix.m[0][0], poseMatrix.m[0][1], // 将 openvr 的坐标系统转换成 tf 的
               -poseMatrix.m[1][2], -poseMatrix.m[1][0], poseMatrix.m[1][1],
               -poseMatrix.m[2][2], -poseMatrix.m[2][0], poseMatrix.m[2][1]);
    tf2::Quaternion q;
    R.getRotation(q);                    // 转换成四元数　ｑ
    tf_.transform.rotation.x = q.getX(); // 设置到 tf_
    tf_.transform.rotation.y = q.getY();
    tf_.transform.rotation.z = q.getZ();
    tf_.transform.rotation.w = q.getW();
    tf_.transform.translation.x = poseMatrix.m[0][3]; //位置  x
    tf_.transform.translation.y = poseMatrix.m[1][3]; // z
    tf_.transform.translation.z = poseMatrix.m[2][3]; // y
}

//
// 发布动态位置
void updateTF()
{
    static tf2_ros::TransformBroadcaster br;             // tf发布器
    //
    // 读取所有设备
    for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
    {
        if (m_pVRSystem->GetControllerState(unDevice, &controllerState, sizeof(controllerState)))
        {
            vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
            switch (trackedDeviceClass)
            {
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD: // 头盔
                vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
                poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking; // 从openvr中获取 M[3][4]
                // 转换成tf
                setTransformStamped();
                tf_.child_frame_id = "HMD";
                tf_.header.stamp = ros::Time::now();
                br.sendTransform(tf_); // 发布tf2
                break;

            case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker: // Tracker（本项目没有这种设备）
                break;

            case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller: // 手柄
                vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, sizeof(controllerState), &trackedDevicePose);
                poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking;
                // 转换成tf
                setTransformStamped();
                auto trackedControllerRole = vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice); // 获得ID
                switch (trackedControllerRole)                                                                 // 判断左右手ID
                {
                case vr::TrackedControllerRole_Invalid:
                    ROS_INFO_STREAM("Tracked Controller Role Invalid");
                    break;
                case vr::TrackedControllerRole_LeftHand:
                    tf_.child_frame_id = "left_controller";
                    tf_.header.stamp = ros::Time::now();
                    br.sendTransform(tf_);
                    break;
                case vr::TrackedControllerRole_RightHand:
                    tf_.child_frame_id = "right_controller";
                    tf_.header.stamp = ros::Time::now();
                    br.sendTransform(tf_);
                    break;
                }
                break;
            }
        }
    }
}

//
// 回调函数
void callback(const sensor_msgs::ImageConstPtr &img_msg_l, const sensor_msgs::ImageConstPtr &img_msg_r)
{
    frame_l = cv_bridge::toCvCopy(img_msg_l, "bgr8")->image;
    frame_r = cv_bridge::toCvCopy(img_msg_r, "bgr8")->image;
    flip(frame_l, frame_l, 0); // 翻转
    flip(frame_r, frame_r, 0);

    if (frame_r.cols != msg_img_width || frame_r.rows != msg_img_height)
    {
        msg_img_width = frame_r.cols;
        msg_img_height = frame_r.rows;
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, msg_img_width, msg_img_height, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL); //TODO*************************
    }

    // 上传图像前获取位姿
    vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
    updateTF(); // 更新位姿

    glClear(GL_COLOR_BUFFER_BIT); // 清除缓存区
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, msg_img_width, msg_img_height, GL_BGR, GL_UNSIGNED_BYTE, frame_l.ptr());
    vr::Texture_t leftEyeTexture = {reinterpret_cast<void *>(intptr_t(textureID)), vr::TextureType_OpenGL, vr::ColorSpace_Auto}; // GLuint
    vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);

    glClear(GL_COLOR_BUFFER_BIT);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, msg_img_height, msg_img_height, GL_BGR, GL_UNSIGNED_BYTE, frame_r.ptr());
    vr::Texture_t rightEyeTexture = {reinterpret_cast<void *>(intptr_t(textureID)), vr::TextureType_OpenGL, vr::ColorSpace_Auto};
    vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);

    //vr::VRCompositor()->PostPresentHandoff();
    // 监视器
    // glBegin(GL_POLYGON);
    //     glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, 0.0f);
    //     glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, 1.0f, 0.0f); // 幕布
    //     glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, 1.0f, 0.0f);
    //     glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, -1.0f, 0.0f);
    //     glEnd();
    glFlush(); // 强制渲染器输出
}

int main(int argc, char **argv)
{
    // 1.启动节点
    ros::init(argc, argv, "VR_bringup");
    ros::NodeHandle node;

    message_filters::Subscriber<sensor_msgs::Image> sub_img_l(node, "/to_VR_bring/imagel", 10);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_r(node, "/to_VR_bring/imager", 10);

    // 同时订阅两幅图像
    // 方法一   完全相同时间
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_img_l, sub_img_r);
    // 方法二  近似时间
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    //message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_img_l, sub_img_r);
    // 方法三   近似时间
    //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_img_l, sub_img_r, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2)); // 指定一个回调函数，就可以实现两个话题数据的同步获
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("VR_bringup ");

    // 初始化VR
    vr::EVRInitError eError = vr::VRInitError_None;
    m_pVRSystem = vr::VR_Init(&eError, vr::VRApplication_Scene); // 初始化openvr
    if (eError != vr::VRInitError_None)
    {
        m_pVRSystem = NULL;
        ROS_ERROR_STREAM("Unable to init VR runtime: %s" + string(vr::VR_GetVRInitErrorAsEnglishDescription(eError)));
    }
    ROS_INFO_STREAM("VR runtime ");

    Init_world();
    glutInit(&argc, argv);
    Init_GL();

    ros::spin();
    return 0;
}