/***************************************************************************************/
// THIS PROGRAM IS FREE SOFTWARE, IS LICENSED MIT
// THIS FILE IS PART OF PROJECT: Immersive simulation environment
// Lib 406
//
// combine_pic.cpp - Combine 2 images that have the same timestamp.
//
// Created on 2021.4.10
// author:菜伟
// email:li_wei_96@163.com
/***************************************************************************************/
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace cv;


Mat frame_l;
Mat frame_r;
Mat X;
Mat submat;
ros::Publisher pub_img;
uint32_t counter = 0;

void callback(const sensor_msgs::ImageConstPtr &img_msg_l, const sensor_msgs::ImageConstPtr &img_msg_r)
{
    frame_l = cv_bridge::toCvCopy(img_msg_l, "rgb8")->image;
    frame_r = cv_bridge::toCvCopy(img_msg_r, "rgb8")->image;
    // 合并 (用于将订阅的图像数据同步，并合并成一个竖向的图像)
    X = Mat(frame_l.rows*2, frame_l.cols, frame_l.type());
    submat = X.rowRange(0, frame_l.rows);
    frame_l.copyTo(submat);
    submat = X.rowRange(frame_l.rows, frame_l.rows*2);
    frame_r.copyTo(submat);
    //cout << X.cols << " " << X.rows <<endl;

    std_msgs::Header header;
    header.seq = counter++;
    header.stamp = ros::Time::now();
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, X);

    sensor_msgs::Image img_msg;     // 转化成 msg
    img_bridge.toImageMsg(img_msg); 
    pub_img.publish(img_msg);
}


int main(int argc, char **argv)
{   
      // 1.启动节点
    ros::init(argc, argv, "dsfadg");
    ros::NodeHandle node;
    pub_img = node.advertise<sensor_msgs::Image>("/ros_vr/display_image", 10);

    // 需要用message_filter容器对两个话题的数据发布进行初始化，这里不能指定回调函数
    message_filters::Subscriber<sensor_msgs::Image> sub_img_l(node, "/gazebo_vr/imager", 10);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_r(node, "/gazebo_vr/imagel", 10);

    // 方法一   完全相同时间
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_img_l, sub_img_r);
    
    // 方法二  近似时间
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    //message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_img_l, sub_img_r);
    
    // 方法三   近似时间
    //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_img_l, sub_img_r, 10);

    // 指定一个回调函数，就可以实现两个话题数据的同步获取
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}