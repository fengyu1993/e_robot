/**
 * 1.这个文件主要是针对普通免驱USB摄像头的代码，最多支持三个摄像头.
 * 2.在使用之前先插上摄像头，不支持代码运行之后再插上并显示新插入的相机图像
 * 3.代码相对简单。基本上也就是opencv采集视频流
 * 4.如果这些代码有用，那它们是xinliang zhong写的，如果没用，那我就不知道是谁写的了。
 *
 * Author: xinliangzhong@foxmail.com
 * Data: 2019.06.10
 */

#ifndef IMAGE_SHOWER_H
#define IMAGE_SHOWER_H

#include <qt5/QtCore/qthread.h>
#include <QCameraInfo>
#include <QCamera>
#include <string>
#include <vector>

/*opencv*/
#include <opencv2/opencv.hpp>

/*Eigen*/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>

using namespace Eigen;


/**
 * 这是单目USB摄像头的主要显示代码。
 * ImageShower类，继承自QThread
 */
class ImageShower : public QThread
{
    Q_OBJECT

public:
    ImageShower();                              //default construct function
    /**
     * 构造函数
     * @param vCameraId 当usb插入之后会有一个id，这里在mainwindow中已经剔除了realsense sense的id编号，剩下的都是usb直接驱动的相机
     */
    ImageShower(std::vector<int> vCameraId);
    virtual ~ImageShower();
    bool init();
    void run();
    bool stop();

    void showCameraDevice();

    QImage toQImage(const cv::Mat &mat); // trans opencv format to QImage format.
    cv::Mat getLeftImage();              // return left image.
    cv::Mat getRightImage();             // return right image.
    cv::Mat getOmniImage();              // return omin image.

    void set_left_port(int index);
    void set_right_port(int index);
    void set_omni_port(int index);

    void set_left_enable(const int &enable);    // open left camera or not.
    void set_right_enable(const int &enable);
    void set_omni_enable(const int &enable);


Q_SIGNALS:
    void leftFrameUpdate();                 // SIGNAL: update left image.
    void rightFrameUpdate();                // SIGNAL: update right image.
    void omniFrameUpdate();                 // SIGNAL: update omni image.

private:

    cv::VideoCapture* cap[3];               // opencv 视频采集流
    cv::VideoCapture* cap_candidate[5];
    cv::Mat img_color[3];                   /*left-right-omni*/
    /*Port*/
    int port[3]; /*left-right-omni*/
    bool is_all_cam_connect;


    /*flag_connect*/
    bool flag_left;
    bool flag_right;
    bool flag_omni;

    int curr_camera_cnt;



};


#endif // IMAGE_SHOWER_H
