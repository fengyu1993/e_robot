#include "dialog.h"
#include "ui_dialog.h"
#include <iostream>
using namespace std;

Dialog::Dialog(QWidget *parent, string url) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    rtsp_url = url;

}


Dialog::~Dialog()
{
    delete ui;
    cout << "delete dialog" << endl;
}


void Dialog::update_label_img()
{
    Mat frame = global_camera->getImage();
    QImage qimg = toQImage(frame);
    ui->label_image->setPixmap(QPixmap::fromImage(qimg));
    ui->label_image->show();

}

void Dialog::run()
{

}

void Dialog::set_rtsp_url(string &url)
{
   rtsp_url = url;
}

void Dialog::open_camera()
{
    global_camera = new GlobalCamera(rtsp_url);
    QObject::connect(global_camera, SIGNAL(frameUpdate()), this, SLOT(update_label_img()));
    global_camera->init();
}

void Dialog::release_camera()
{
    global_camera->setIsTransImage(false);
}

QImage Dialog::toQImage(const cv::Mat& mat)
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


void Dialog::on_button_platform_up_clicked()
{

}

void Dialog::on_button_platform_left_clicked()
{

}

void Dialog::on_button_platform_right_clicked()
{

}

void Dialog::on_button_platform_down_clicked()
{

}

void Dialog::on_button_platform_stop_clicked()
{

}
