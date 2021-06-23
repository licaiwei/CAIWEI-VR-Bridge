/***************************************************************************************/
// THIS PROGRAM IS FREE SOFTWARE, IS LICENSED MIT
// THIS FILE IS PART OF PROJECT: Immersive simulation environment
// Lib 406
//
// vr_device_states_controller.cpp - Subscribe VR device TF to set gazebo model pose
//
// Created on 2021.4.9
// author:菜伟
// email:li_wei_96@163.com
/***************************************************************************************/
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/Pose.h> //ROS标准topic
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>                     // TF相关
#include <tf/transform_listener.h>     // 接受TF
#include <tf/transform_broadcaster.h>  // 发送TF
#include <tf2/LinearMath/Quaternion.h> // tf2相关
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo_msgs/ModelState.h> //
#include <iostream>

gazebo_msgs::ModelState model_state;
geometry_msgs::TransformStamped transformStamped;

void setPose()
{
	model_state.pose.position.x = transformStamped.transform.translation.x;
	model_state.pose.position.y = transformStamped.transform.translation.y;
	model_state.pose.position.z = transformStamped.transform.translation.z;
	model_state.pose.orientation.w = transformStamped.transform.rotation.w;
	model_state.pose.orientation.x = transformStamped.transform.rotation.x;
	model_state.pose.orientation.y = transformStamped.transform.rotation.y;
	model_state.pose.orientation.z = transformStamped.transform.rotation.z;
}

int main(int argc, char **argv)
{
	//
	// 启动节点
	ros::init(argc, argv, "vr_HMD_states_controllor"); // 节点名
	ros::NodeHandle node;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer); // 创建一个listener对象，一旦创建就开始接收，缓冲10秒。通常将这个对象创建为类的成员变量
	ros::Duration(0.5).sleep();

	ros::Publisher pub = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

	ros::Rate rate(50);
	while (node.ok())
	{
		try
		{
			transformStamped = tfBuffer.lookupTransform("World", "HMD", ros::Time(0));
			model_state.model_name = "vr_HMD";
			setPose();
			pub.publish(model_state);
		}
		catch (tf2::TransformException &ex)
		{
			//ROS_WARN("%s",ex.what());
		 	ROS_INFO_STREAM("Waiting for HMD TF");
		 	ros::Duration(1.0).sleep();
		 	continue;
		}

		try
		{
			transformStamped = tfBuffer.lookupTransform("World", "right_controller", ros::Time(0));
			model_state.model_name = "vr_controller_r";
			setPose();
			pub.publish(model_state);
		}
		catch (tf2::TransformException &ex)
		{
			//ROS_WARN("%s",ex.what());
		 	ROS_INFO_STREAM("Waiting for right_controller TF");
		 	//ros::Duration(1.0).sleep();
		 	continue;
		}

		try
		{
			transformStamped = tfBuffer.lookupTransform("World", "left_controller", ros::Time(0));
			model_state.model_name = "vr_controller_l";
			setPose();
			pub.publish(model_state);
		}
		catch (tf2::TransformException &ex)
		{
			//ROS_WARN("%s",ex.what());
		 	ROS_INFO_STREAM("Waiting for left_controller TF");
		 	//ros::Duration(1.0).sleep();
		 	continue;
		}
		rate.sleep();
	}
	return 0;
}