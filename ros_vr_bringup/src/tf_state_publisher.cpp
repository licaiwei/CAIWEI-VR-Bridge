/***************************************************************************************/
// THIS FILE IS PART OF PROJECT: Immersive simulation environment
// Lib 406
//
// tf_state_publisher.cpp - Convert the coordinate system of the device in TF
//
// Created on 2021.4.8
// author:菜伟
// email:li_wei_96@163.com
/***************************************************************************************/

#include <openvr.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std;

vr::TrackedDevicePose_t trackedDevicePose; // 设备姿态（复用）
vr::VRControllerState_t controllerState;   // 控制器状态(复用)
vr::HmdMatrix34_t poseMatrix;              // 设备姿态（复用）
geometry_msgs::TransformStamped tf_;       // 利用标准的msg（复用）

//
// 将全局变量 poseMatrix 转化成 tf_。用于发布设备位姿
void setTransformStamped()
{
    tf2::Matrix3x3 R;
    R.setValue(-poseMatrix.m[0][2], -poseMatrix.m[0][0], poseMatrix.m[0][1], // 将 openvr 的坐标系统转换成 tf 的
               -poseMatrix.m[1][2], -poseMatrix.m[1][0], poseMatrix.m[1][1], 
               -poseMatrix.m[2][2], -poseMatrix.m[2][0], poseMatrix.m[2][1]);
    tf2::Quaternion q;
    R.getRotation(q); // 转换成四元数　ｑ
    tf_.transform.rotation.x = q.getX(); // 设置到 tf_
    tf_.transform.rotation.y = q.getY();
    tf_.transform.rotation.z = q.getZ();
    tf_.transform.rotation.w = q.getW();
    tf_.transform.translation.x = poseMatrix.m[0][3]; //位置  x
    tf_.transform.translation.y = poseMatrix.m[1][3]; // z
    tf_.transform.translation.z = poseMatrix.m[2][3]; // y
}

int main(int argc, char **argv)
{
    //
    // 1.启动节点
    ros::init(argc, argv, "vrDevices_states_publisher");
    ros::NodeHandle node;
    tf2_ros::TransformBroadcaster br; // 创建发布器
    tf2_ros::StaticTransformBroadcaster brStatic; // 创建静态tf，直接转换 openvr 的世界坐标系统
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("vrDevices_states_publisher ");

    //
    // 2.初始化VR
    vr::IVRSystem *m_pVRSystem;
    vr::EVRInitError eError = vr::VRInitError_None;
    m_pVRSystem = vr::VR_Init(&eError, vr::VRApplication_Scene);
    if (eError != vr::VRInitError_None)
    {
        m_pVRSystem = NULL;
        ROS_ERROR_STREAM("Unable to init VR runtime: %s" + string(vr::VR_GetVRInitErrorAsEnglishDescription(eError)));
        return 0;
    }
    ROS_INFO_STREAM("VR runtime ");

    //
    // 3.转换世界坐标系
    tf_.header.frame_id = "World";    // 设置frame_ID
    tf_.child_frame_id = "vr_world";
    tf2::Matrix3x3 r_;
    r_.setValue( 0, 0, 1,
                1, 0, 0,
                0, 1, 0);
    tf2::Quaternion q_;
    r_.getRotation(q_);
    tf_.transform.rotation.x = q_.getX();
    tf_.transform.rotation.y = q_.getY();
    tf_.transform.rotation.z = q_.getZ();
    tf_.transform.rotation.w = q_.getW();
    tf_.header.stamp = ros::Time::now();
    brStatic.sendTransform(tf_);
    
    //
    // 4.发布动态位置
    ros::Rate rate(30);
    tf_.header.frame_id = "vr_world";    // 设置frame_ID
    while (node.ok())
    {
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
        rate.sleep();
    }
    return 0;
}