
/**
 * 1.这个文件主要是针对欧克（海康威视机芯）相机的获取图像的代码，一般来说不需要做修改。
 * 2.另外相机自带云台，云台控制是通过485控制的，其中通信协议在serial_common模块中
 * 3.购买地址:https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8dxy50re&id=528690430878&_u=a1i9rt826ba2
 * 4.如果这些代码有用，那它们是xinliang zhong写的，如果没用，那我就不知道是谁写的了。
 *
 * Author: xinliangzhong@foxmail.com
 * Data: 2019.06.10
 */

#ifndef GLOBAL_CAMERA_H
#define GLOBAL_CAMERA_H

/*C++ standard*/
#include <string>
#include <iostream>

/*Qt*/

#include <qt5/QtCore/qthread.h>
#include <QCameraInfo>
#include <QCamera>

/*opencv*/
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

/**
 * GlobalCamera class.
 * 继承自QTHread，因为相机作为一个单独的线程一直在工作，当有新图像到来时给主程序信号并更新显示界面的图像.
 */
class GlobalCamera : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief GlobalCamera The construct function, you must set the url.(defult = "")
     * @param url The rtsp url of the camera.
     */
    explicit GlobalCamera(string url = "");
    virtual ~GlobalCamera();
    bool init();
    void run();
    bool stop();

    /**
     * 与外部联系的函数，返回相机当前图像
     * @return 当前图像
     */
    cv::Mat getImage();
    void setIsTransImage(bool is_trans);

    /**
     * 将opencv存储的图像格式转成qt的QImage，因为QT显示需要QImage.
     * trans opencv format to qt format.
     * @param mat opencv format
     * @return
     */
    QImage toQImage(const cv::Mat& mat);

    /**
     * 设置相机的开关
     * Set the camera status.
     * @param is_on true: open the camera and get image stream
     * @param is_on false: do not update the image stream to reduce the cpu and memory.
     */
    void set_gcamera_on_off(bool is_on);

Q_SIGNALS:
    void gframeUpdate();    //信号:发出新图到来，需要更新图像的信号

private:

    cv::VideoCapture *cap_global;   //OpenCV 视频采集类.
    bool is_trans_image;            //May be not used.
    string url;                     //rtsp url.rtsp流，相机并不是usb接口，而是内部会发出一个rtsp数据流，opencv根据数据流的地址读取图像
    cv::Mat frame;                  //image frame.
    bool is_on;                     //是否开启的标志
    bool is_init;                   //是否初始化的标志

};


#endif // GLOBAL_CAMERA_H
