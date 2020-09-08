#include "image_shower.h"

#include <string>
#include <sstream>
#include <QImage>
#include <QMessageBox>

using namespace std;
using namespace cv;
using namespace Qt;
using namespace Eigen;

/**
 * 初始化一些标志位
 */
ImageShower::ImageShower()
{

    is_all_cam_connect = false;
    flag_left = false;
    flag_right = false;
    flag_omni = false;
    curr_camera_cnt = 0;

}
/**
 * 构造函数
 * @param vCameraId usb相机的端口ID
 */
ImageShower::ImageShower(vector<int> vCameraId)
{
    curr_camera_cnt = vCameraId.size();
    for(int i = 0; i < vCameraId.size(); i++)
    {
        if(i == 0)
            flag_left=true;
        else if(i == 1)
            flag_right=true;
        else if(i == 2)
            flag_omni=true;
        port[i] = vCameraId[i];
        cap[i] = new VideoCapture(port[i]);
        cout << "camera_mono: " << i+1 << endl;
    }
}


ImageShower::~ImageShower()
{
    terminate();
    wait();
}

bool ImageShower::init()
{
    start();

    return true;
}


bool ImageShower::stop()
{
    terminate();

    return true;
}

/**
 * 完成对相机的视频流采集工作，并且根据flag决定是否更新对应的相机流
 */
void ImageShower::run()
{

    while(true)
    {
        for(int i = 0; i < curr_camera_cnt; ++i)
        {

            if(cap[i]->isOpened()&&(cap[i]!= nullptr))
            {

                if(i == 0 && flag_left)
                {
                    cap[i]->read(img_color[i]);
                    Q_EMIT leftFrameUpdate();
                }
                else if(i == 1 && flag_right)
                {
                    cap[i]->read(img_color[i]);
                    Q_EMIT rightFrameUpdate();
                }
                else if(i == 2 && flag_omni)
                {
                    cap[i]->read(img_color[i]);
                    Q_EMIT omniFrameUpdate();
                }

            }
        }

    }
}

/**
 * QCamera 对usb相机的id进行扫描
 */
void ImageShower::showCameraDevice()
{
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    cout << "We detect camera nums: " << cameras.size() << endl;
    for(int i = 0; i < cameras.size(); ++i)
    {
        cout << "id: " << cameras.at(i).description().toStdString() <<  endl;
        cout << "id: " << cameras.at(i).deviceName().toStdString() <<  endl;
    }
}

/**
 * trans opencv mat to QImgae.
 * @param mat
 * @return
 */
QImage ImageShower::toQImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        //        cv::cvtColor(mat,mat,CV_BGR2RGB);
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        return QImage();
    }
}

/**
 * 获取图像
 * @return
 */
cv::Mat ImageShower::getLeftImage()
{
    cv::Mat img = img_color[0].clone();
    return img;
}

cv::Mat ImageShower::getRightImage()
{
    cv::Mat img = img_color[1].clone();
    return img;
}

cv::Mat ImageShower::getOmniImage()
{
    cv::Mat img = img_color[2].clone();
    return img;
}

void ImageShower::set_left_port(int index)
{
    port[0] = index;
    cap[0] = new VideoCapture(index);
}

void ImageShower::set_right_port(int index)
{
    port[1] = index;
    //    cap[1] = new VideoCapture(index);
}

void ImageShower::set_omni_port(int index)
{
    port[2] = index;
    //    cap[2] = new VideoCapture(index);
}

/**
 * @param enable flag:打开或者关闭
 */
void ImageShower::set_left_enable(const int &enable)
{
    flag_left = enable > 0 ? 1:0;
    cout << "flag left" << flag_left << endl;
}

void ImageShower::set_right_enable(const int &enable)
{
    flag_right = enable > 0 ? 1:0;
}

void ImageShower::set_omni_enable(const int &enable)
{
    flag_omni = enable > 0 ? 1:0;
}
