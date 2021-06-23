/***************************************************************************************/
// THIS FILE IS PART OF PROJECT: Immersive simulation environment
// Lib 406
//
// submit_image_node.cpp - Display your camera frame in HMD
//
// Created on 2021.4.8
// author:菜伟
// email:li_wei_96@163.com
/***************************************************************************************/
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
	
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <GL/glut.h>
#include <SDL2/SDL_opengl.h>
#include <GL/glu.h>

#include <openvr.h>
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    //
    // 1.初始化VR
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem *m_pVRSystem = vr::VR_Init( &eError, vr::VRApplication_Scene );

    //
    // 2.初始化 GL
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE); // 窗口类型（颜色|单/双缓存）
    //glutInitWindowPosition(0, 0);
    glutInitWindowSize(640, 480);
    glutCreateWindow("OpenGL 3D View");
    glClearColor(0.2, 0.3, 0.3, 1.0);   // 背景颜色，清除缓冲区时填充的颜色
    glMatrixMode(GL_PROJECTION);        // 表示下面的操作为设置投影矩阵         参数有：GL_PROJECTION, GL_MODELVIEW, GL_TEXTURE
    glOrtho(-1.1, 1.1, -1.1, 1.1, -15, 15);       // (left, right, bottom, top, near, far)
    glMatrixMode(GL_MODELVIEW);         // 设置模型视景矩阵
    gluLookAt(0, 0, 14, 0, 0, 0, 0, 1, 0); // 摄像机的位置、规定观察点、规定摄像机的垂直朝向
    glEnable(GL_TEXTURE_2D);

    GLuint textureID;   // 生成纹理 ID
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);   // 缩小过滤，
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // 放大过滤，
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);  // S方向
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);  // T方向
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 512, 256, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL);

    Mat frame;
    Mat image_cut, image_cut2;
    Size srcSize = Size(512, 512);  //填入任意指定尺寸

    //
    // 3.打开一个默认的相机
    VideoCapture capture(0);
    if(!capture.isOpened()) 
        return -1;
    while(1)
    {
        // 获取 mat
        capture >> frame;   //从相机读取新一帧
        flip(frame, frame, 0);
        resize(frame, frame, srcSize, 0, 0, INTER_LINEAR);
        vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
        vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
        
        // GL纹理
        glClear(GL_COLOR_BUFFER_BIT); // 清除缓存区
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_cut.cols, image_cut.rows, GL_BGR, GL_UNSIGNED_BYTE, image_cut.ptr());
		vr::Texture_t leftEyeTexture = {reinterpret_cast<void*>(intptr_t(textureID)), vr::TextureType_OpenGL, vr::ColorSpace_Auto }; // GLuint
        vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
        glClear(GL_COLOR_BUFFER_BIT); // 清除缓存区
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_cut2.cols, image_cut2.rows, GL_BGR, GL_UNSIGNED_BYTE, image_cut2.ptr());
		vr::Texture_t rightEyeTexture = {reinterpret_cast<void*>(intptr_t(textureID)), vr::TextureType_OpenGL, vr::ColorSpace_Auto };
		vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
        vr::VRCompositor()->PostPresentHandoff();
        // 画出监视器
        glBegin(GL_POLYGON);
            glTexCoord2f(0.0f, 0.0f);   glVertex3f(-1.0f, -1.0f, 0.0f);
            glTexCoord2f(0.0f, 1.0f);   glVertex3f(-1.0f,  1.0f, 0.0f); // 幕布
            glTexCoord2f(1.0f, 1.0f);	glVertex3f( 1.0f,  1.0f, 0.0f);
            glTexCoord2f(1.0f, 0.0f);   glVertex3f( 1.0f, -1.0f, 0.0f);
            glEnd();
        glFlush(); // 强制渲染器输出
    }
    glDisable(GL_TEXTURE_2D);
}