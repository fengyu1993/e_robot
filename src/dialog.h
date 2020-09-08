/**
 * 理论上这个没什么用了 只是当时调试用了
 */
#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

#include "global_camera/global_camera.h"

using namespace cv;
using namespace std;

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0, string url="");
    ~Dialog();

    QImage toQImage(const cv::Mat &mat);
    void set_rtsp_url(string &url);

    void run();

    void open_camera();

    void release_camera();



private Q_SLOTS:

    void update_label_img();

    void on_button_platform_up_clicked();

    void on_button_platform_left_clicked();

    void on_button_platform_right_clicked();

    void on_button_platform_down_clicked();

    void on_button_platform_stop_clicked();

private:
    Ui::Dialog *ui;
    string rtsp_url;

    GlobalCamera *global_camera;

};

#endif // DIALOG_H
