#include "global_camera.h"

/**
 * Construct function
 * @param url rtsp stream. eg.rtsp://admin:admin123@192.168.3.64:554/h264/ch1/main/av_stream
 * @param url 192.168.3.64 相机ip地址，使用前建议先用相机IP扫描工具检测一下IP.
 */
GlobalCamera::GlobalCamera(string url)
{
    cap_global = new VideoCapture();
    this->url = url;
    is_trans_image = true;
    cout << "url:" << this->url << endl;
    is_on = true;
    is_init = false;
}

GlobalCamera::~GlobalCamera()
{
    delete cap_global;
    terminate();

}

bool GlobalCamera::init()
{
    bool flag = cap_global->open(url);
    if(flag)
        start();
    else
        return false;

    return true;
}

bool GlobalCamera::stop()
{
    if(cap_global->isOpened())
    {
        terminate();

        cap_global->release();
    }


    return true;
}

/**
 * 循环采集图像并将更新信号发射出去
 */
void GlobalCamera::run()
{

    while(true)
    {

        {
            if(cap_global->isOpened())
            {
                    if(is_on)//打开标志位，如果关闭的话就不更新图像
                    {
                        cap_global->read(frame);
                        if(!frame.empty() && (frame.rows == 1080) && (frame.cols == 1920))//主要检测图像是否完整，避免丢帧错帧
                        {
                            // cout << "rows = " << frame.rows << " cols = " << frame.cols << endl;
                            //cv::resize(frame, frame, Size(frame.cols/2, frame.rows/2));

                            Q_EMIT gframeUpdate(); //将图像更新信号发射出去
                        }
                    }
            }
            else
            {
                cout << "some error, retry" << endl;
                cap_global->open(url);
            }
        }
    }
}

/**
 * return the copy of current image.
 * @return current image
 */
cv::Mat GlobalCamera::getImage()
{
    return frame.clone();
}

/**
 * Set the camera on or off.
 * if on. the mainwindow(ui) can refresh the image.
 * if off. the mainwindow can not refresh the image, and will keep the last image in the label.
 * @param is_on
 */
void GlobalCamera::set_gcamera_on_off(bool is_on)
{
    if(!is_init)
    {
        is_init = true;
        init();
    }
    this->is_on = is_on;
}


void GlobalCamera::setIsTransImage(bool is_trans)
{
    is_trans_image = is_trans;
}

/**
 * A useful function trans opencv mat to QImage.
 * @param mat
 * @return
 */
QImage GlobalCamera::toQImage(const cv::Mat& mat)
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
