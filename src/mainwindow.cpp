/**
 * 可以理解为这里是很多函数的入口 也是之前很多类与qt界面的接口，但凡要与ui交互的都需要在这里实现
 * 代码里边可能会存在一些测试文件 所以看的时候留心一点
 *
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "librealsense2/rs.hpp"
#include <iostream>
#include <QCamera>
#include <QTimer>
#include <vector>
#include <cstdlib>
#include <unistd.h>

#include "socket_robot/utility.h"
#include "socket_robot/socket_robot.h"
#include "serial_common/serial_common.h"

using namespace std;

extern rs2::frame global_depth;
extern rs2::frame global_depth_1;
extern rs2::frame global_depth_2;

/*192.168.2.64是相机的IP地址，也是局域网地址，如果换了网络环境也记得更换，用设备扫描软件扫描或者进行修改*/
const string gcamera_rtsp = "rtsp://admin:admin123@192.168.2.64:554/h264/ch1/main/av_stream";
const QString ip_address = "192.168.2.200";//远程终端ip地址

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    serial_is_ok = false;
    ui->setupUi(this);
    setWindowIcon(QIcon(":/imgs/icon.png"));
    /*init index of cameras*/
    vector<QString> mono_cameras_index = get_camera_id_list();
    vector<int> camera_ids;
    for(auto &id : mono_cameras_index)
    {
        ui->left_port_index->addItem(id);
        camera_ids.push_back(QString(id).toInt());
    }
    /*mono_camera*/
    image_shower = new ImageShower(camera_ids);
    /*socket_robot*/
    socket_robot = new SocketRobot();

    is_dialog_launch = false;
    /*Global Camera*/
    gcamera = new GlobalCamera(gcamera_rtsp);

    /*定时器_按时请求数据*/
    QTimer * ask_data = new QTimer(this);
    Flag_Timeout = 1;
//    ask_data->start(ASK_DATA_PERIOD); //

    /*watch dog*/
    watchdog = new UIWatchDog(this);   

    /*ping check*/
    pingcheck = new PingCheck(ip_address);
    pingcheck->init();

    /*这里其实是连接各个子模块与UI界面的信号与槽函数*/
    QObject::connect(gcamera, SIGNAL(gframeUpdate()), this, SLOT(update_gcamera()));
    QObject::connect(&image_processor, SIGNAL(frameUpdate()), this, SLOT(update_label_realsense()));
    QObject::connect(ui->label_realsense_rgb, SIGNAL(moved_send_position(int, int)), this, SLOT(show_depth(int,int)));
    QObject::connect(image_shower, SIGNAL(leftFrameUpdate()), this, SLOT(update_left_image()));
    QObject::connect(image_shower, SIGNAL(rightFrameUpdate()), this, SLOT(update_right_image()));
    QObject::connect(image_shower, SIGNAL(omniFrameUpdate()), this, SLOT(update_omni_image()));
    QObject::connect(ui->label_realsense_rgb_2, SIGNAL(moved_send_position(int, int)), this, SLOT(show_depth_1(int,int)));
    QObject::connect(ui->label_realsense_rgb_3, SIGNAL(moved_send_position(int, int)), this, SLOT(show_depth_2(int,int)));

    QObject::connect(&image_processor, SIGNAL(navigation()), this, SLOT(send_navi_waypoints()));
    QObject::connect(&image_processor, SIGNAL(navigation_order()), this, SLOT(send_navi_waypoints_order()));
    QObject::connect(&image_processor, SIGNAL(auto_navigation()), this, SLOT(send_auto_navi_waypoints()));
    QObject::connect(&image_processor, SIGNAL(auto_navigation_order()), this, SLOT(send_auto_navi_waypoints_order()));
    QObject::connect(&image_processor, SIGNAL(grasp_object()), this, SLOT(send_grasp_positionmatrix()));
    QObject::connect(&image_processor, SIGNAL(grasp_object_order()), this, SLOT(send_grasp_positionmatrix_order()));



    QObject::connect(socket_robot, SIGNAL(receive_arm1_joint_angle()), this, SLOT(update_arm1_joint_angle()));
    QObject::connect(socket_robot, SIGNAL(receive_arm2_joint_angle()), this, SLOT(update_arm2_joint_angle()));
    QObject::connect(socket_robot, SIGNAL(receive_arm1_joint_status()), this, SLOT(update_arm1_joint_status()));
    QObject::connect(socket_robot, SIGNAL(receive_arm2_joint_status()), this, SLOT(update_arm2_joint_status()));
    QObject::connect(socket_robot, SIGNAL(receive_vehicle_speed()), this, SLOT(update_vehicle_speed()));
    QObject::connect(socket_robot, SIGNAL(receive_arm1_actuator_posture()), this, SLOT(update_arm1_actuator_posture()));
    QObject::connect(socket_robot, SIGNAL(receive_arm2_actuator_posture()), this, SLOT(update_arm2_actuator_posture()));
    QObject::connect(socket_robot, SIGNAL(receive_error_code()), this, SLOT(update_error_code()));
    QObject::connect(socket_robot, SIGNAL(receive_all_data()), this, SLOT(update_all_data()));


    QObject::connect(ui->arm1_slider1, SIGNAL(sliderReleased()), this, SLOT(send_arm1_joint_angle()));
    QObject::connect(ui->arm1_slider2, SIGNAL(sliderReleased()), this, SLOT(send_arm1_joint_angle()));
    QObject::connect(ui->arm1_slider3, SIGNAL(sliderReleased()), this, SLOT(send_arm1_joint_angle()));
    QObject::connect(ui->arm1_slider4, SIGNAL(sliderReleased()), this, SLOT(send_arm1_joint_angle()));
    QObject::connect(ui->arm1_slider5, SIGNAL(sliderReleased()), this, SLOT(send_arm1_joint_angle()));
    QObject::connect(ui->arm1_slider6, SIGNAL(sliderReleased()), this, SLOT(send_arm1_joint_angle()));

    QObject::connect(ui->arm2_slider1, SIGNAL(sliderReleased()), this, SLOT(send_arm2_joint_angle()));
    QObject::connect(ui->arm2_slider2, SIGNAL(sliderReleased()), this, SLOT(send_arm2_joint_angle()));
    QObject::connect(ui->arm2_slider3, SIGNAL(sliderReleased()), this, SLOT(send_arm2_joint_angle()));
    QObject::connect(ui->arm2_slider4, SIGNAL(sliderReleased()), this, SLOT(send_arm2_joint_angle()));
    QObject::connect(ui->arm2_slider5, SIGNAL(sliderReleased()), this, SLOT(send_arm2_joint_angle()));
    QObject::connect(ui->arm2_slider6, SIGNAL(sliderReleased()), this, SLOT(send_arm2_joint_angle()));

    QObject::connect(ask_data, SIGNAL(timeout()), this, SLOT(handleTimeout_askdata()));
    QObject::connect(pingcheck, SIGNAL(pingfail()), this, SLOT(pingfail_handle()));



    arm1_last_pose = Arm_pose(get_arm1_values());
    arm1_curr_pose = arm1_last_pose;
    arm2_last_pose = Arm_pose(get_arm2_values());
    arm2_curr_pose = arm2_last_pose;

    scan_realsense_list();

    // ui默认显示
    ui->tabWidget_3->setCurrentIndex(3);
    ui->tabWidget_2->setCurrentIndex(0);
    ui->tabWidget->setCurrentIndex(4);
    ui->gcamera->setCurrentIndex(0);
    ui->button_close_all_cameras->setDisabled(1);
    ui->flywheel_Out->setEnabled(1);
    ui->flywheel_In->setEnabled(0);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete image_shower;
    delete socket_robot;
    delete gcamera;
    delete watchdog;
    delete pingcheck;
}


void MainWindow::on_button_launch_camera_clicked()
{
    image_processor.init(); //start rgbd camera

    image_shower->init(); //start mono_camera.

    gcamera->init(); //start global_camera

    watchdog->startWatchDog();

    ui->button_launch_camera->setDisabled(1);

    ui->button_close_all_cameras->setEnabled(1);

}
void MainWindow::on_is_estimate_ground_plane_clicked(bool checked)//估计地面的那个按钮
{
    //run the estimate plane;
    image_processor.setGroundPlaneEstimateFlag(checked);//将估计地面的那个按钮设置为不可用
    image_processor.estimateGroundPlane();

}

void MainWindow::on_set_waypoints_clicked(bool checked)
{
    if(checked)
    {
        is_set_waypoints = true;
        image_processor.setWayPoints();
    }

}

void MainWindow::on_set_grasppoints_clicked(bool checked)
{
    if(checked)
    {
        is_set_grasppoints = true;
        image_processor.setgraspPoints();
    }

}

void MainWindow::on_set_autowaypoints_clicked(bool checked)
{
    if(checked)
    {
        is_set_autowaypoints = true;
        image_processor.autoWayPoints();
    }

}




void MainWindow::send_navi_waypoints()
{


    std::vector<Vector2d> nav_waypoints_cam_cm=(image_processor.obtain_navigation_waypoints());


     std::string temp;
     for (int i = 0; i < nav_waypoints_cam_cm.size(); ++i)
     {

        for(int j=0;j<2;++j)
        {
       string v1 = Utility::paddingZero(nav_waypoints_cam_cm[i](j)*100+5000);//nav_waypoints_cam_cm danwei shi m
         temp=temp+v1;
        }


     }

     cout<<nav_waypoints_cam_cm.size()<<endl;
     int n=8*2*nav_waypoints_cam_cm.size();

         string msg_info = Utility::toString<int>(HtoS_send_naviWayPoints) + Utility::paddingZero(n) + temp;
         cout << "标识符+长度: " << HtoS_send_naviWayPoints << Utility::paddingZero(n)<<endl;
         cout << "     data: " << temp << endl;
         socket_robot->send_msg(msg_info);

}
void MainWindow::send_navi_waypoints_order()
{

    string msg_info = Utility::toString<int>(HtoS_naviWayPoints_Start)+"1";
    cout << "标识符+长度: " << HtoS_naviWayPoints_Start << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}






void MainWindow::send_auto_navi_waypoints()
{


    std::vector<Vector2d> nav_waypoints_cam_cm=(image_processor.obtain_auto_navigation_waypoints());


     std::string temp;
     for (int i = 0; i < nav_waypoints_cam_cm.size(); ++i)
     {

        for(int j=0;j<2;++j)
        {
       string v1 = Utility::paddingZero(nav_waypoints_cam_cm[i](j)*100+5000);//danwei shi m
         temp=temp+v1;
        }


     }

 //    cout<<nav_waypoints_cam_cm.size()<<endl;
     int n=8*2*nav_waypoints_cam_cm.size();

         string msg_info = Utility::toString<int>(HtoS_send_autoWayPoints) + Utility::paddingZero(n) + temp;
         cout << "标识符+长度: " << HtoS_send_autoWayPoints << Utility::paddingZero(n)<<endl;
         cout << "     data: " << temp << endl;
         socket_robot->send_msg(msg_info);

}

void MainWindow::send_auto_navi_waypoints_order()
{

    string msg_info = Utility::toString<int>(HtoS_autoWayPoints_Start)+"1";
    cout << "标识符+长度: " << HtoS_autoWayPoints_Start << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}



void MainWindow::send_grasp_positionmatrix()
{


    std::vector<double> nav_waypoints_cam_cm=(image_processor.obtain_grasp_object_position( ));


     std::string temp;
     for (int i = 0; i < nav_waypoints_cam_cm.size(); ++i)
     {


       string v1 = Utility::paddingZero(nav_waypoints_cam_cm[i]*100+5000);//danwei shi mi
         temp=temp+v1;



     }

 //    cout<<nav_waypoints_cam_cm.size()<<endl;
     int n=8*nav_waypoints_cam_cm.size();

         string msg_info = Utility::toString<int>(HtoS_send_graspPoints) + Utility::paddingZero(n) + temp;
         cout << "标识符+长度: " << HtoS_send_graspPoints << Utility::paddingZero(n)<<endl;
         cout << "     data: " << temp << endl;
         socket_robot->send_msg(msg_info);

}


void MainWindow::send_grasp_positionmatrix_order()
{

    string msg_info = Utility::toString<int>(HtoS_graspPoints_Start)+"1";
    cout << "标识符+长度: " << HtoS_graspPoints_Start << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}





/*
string mainwindow::data_to_string()
{
    string temp;
    for (int i = 0; i < nav_waypoints_cam_cm.size(); ++i)
    {

       for(int j=0;j<2;++j)
       {
      string v1 = Utility::paddingZero(nav_waypoints_cam_cm[i](j)*100+5000);
        temp=temp+v1;
       }


    }

    return temp;

}

string MainWindow::data_to_biaoshi()
{

    int n=8*3*nav_waypoints_cam_cm.size();
    string v2 = Utility::paddingZero(n);
    string m="1007";
    return m+v2;

}



*/








void MainWindow::update_arm1_joint_angle()
{
//    socket_robot->arm1_pose.joint1;
    ui->arm1_slider1->setValue(socket_robot->StoH_ARM1_Joint_Angle[0]);
    ui->arm1_slider2->setValue(socket_robot->StoH_ARM1_Joint_Angle[1]);
    ui->arm1_slider3->setValue(socket_robot->StoH_ARM1_Joint_Angle[2]);
    ui->arm1_slider4->setValue(socket_robot->StoH_ARM1_Joint_Angle[3]);
    ui->arm1_slider5->setValue(socket_robot->StoH_ARM1_Joint_Angle[4]);
    ui->arm1_slider6->setValue(socket_robot->StoH_ARM1_Joint_Angle[5]);
//
    ui->arm1_joint_angle_1->setText(QString::number(socket_robot->StoH_ARM1_Joint_Angle[0],10,0));
    ui->arm1_joint_angle_2->setText(QString::number(socket_robot->StoH_ARM1_Joint_Angle[1],10,0));
    ui->arm1_joint_angle_3->setText(QString::number(socket_robot->StoH_ARM1_Joint_Angle[2],10,0));
    ui->arm1_joint_angle_4->setText(QString::number(socket_robot->StoH_ARM1_Joint_Angle[3],10,0));
    ui->arm1_joint_angle_5->setText(QString::number(socket_robot->StoH_ARM1_Joint_Angle[4],10,0));
    ui->arm1_joint_angle_6->setText(QString::number(socket_robot->StoH_ARM1_Joint_Angle[5],10,0));
}

void MainWindow::update_arm2_joint_angle()
{
//    socket_robot->arm1_pose.joint1;
    ui->arm2_slider1->setValue(socket_robot->StoH_ARM2_Joint_Angle[0]);
    ui->arm2_slider2->setValue(socket_robot->StoH_ARM2_Joint_Angle[1]);
    ui->arm2_slider3->setValue(socket_robot->StoH_ARM2_Joint_Angle[2]);
    ui->arm2_slider4->setValue(socket_robot->StoH_ARM2_Joint_Angle[3]);
    ui->arm2_slider5->setValue(socket_robot->StoH_ARM2_Joint_Angle[4]);
    ui->arm2_slider6->setValue(socket_robot->StoH_ARM2_Joint_Angle[5]);
//
    ui->arm2_joint_angle_1->setText(QString::number(socket_robot->StoH_ARM2_Joint_Angle[0],10,0));
    ui->arm2_joint_angle_2->setText(QString::number(socket_robot->StoH_ARM2_Joint_Angle[1],10,0));
    ui->arm2_joint_angle_3->setText(QString::number(socket_robot->StoH_ARM2_Joint_Angle[2],10,0));
    ui->arm2_joint_angle_4->setText(QString::number(socket_robot->StoH_ARM2_Joint_Angle[3],10,0));
    ui->arm2_joint_angle_5->setText(QString::number(socket_robot->StoH_ARM2_Joint_Angle[4],10,0));
    ui->arm2_joint_angle_6->setText(QString::number(socket_robot->StoH_ARM2_Joint_Angle[5],10,0));
}

void MainWindow::update_arm1_joint_status()
{
    //joint 1
    if (socket_robot->StoH_ARM1_Joint_Status[0] == 1)
    {
        ui->arm1_flag1->setText("Y");
    }
    else
    {
        ui->arm1_flag1->setText("N");
    }
    // joint 2
    if (socket_robot->StoH_ARM1_Joint_Status[1] == 1)
    {
        ui->arm1_flag2->setText("Y");
    }
    else
    {
        ui->arm1_flag2->setText("N");
    }
    // joint 3
    if (socket_robot->StoH_ARM1_Joint_Status[2] == 1)
    {
        ui->arm1_flag3->setText("Y");
    }
    else
    {
        ui->arm1_flag3->setText("N");
    }
    // joint 4
    if (socket_robot->StoH_ARM1_Joint_Status[3] == 1)
    {
        ui->arm1_flag4->setText("Y");
    }
    else
    {
        ui->arm1_flag4->setText("N");
    }
    // joint 5
    if (socket_robot->StoH_ARM1_Joint_Status[4] == 1)
    {
        ui->arm1_flag5->setText("Y");
    }
    else
    {
        ui->arm1_flag5->setText("N");
    }
    // joint 6
    if (socket_robot->StoH_ARM1_Joint_Status[5] == 1)
    {
        ui->arm1_flag6->setText("Y");
    }
    else
    {
        ui->arm1_flag6->setText("N");
    }
}

void MainWindow::update_arm2_joint_status()
{
    //joint 1
    if (socket_robot->StoH_ARM2_Joint_Status[0] == 1)
    {
        ui->arm2_flag1->setText("Y");
    }
    else
    {
        ui->arm2_flag1->setText("N");
    }
    // joint 2
    if (socket_robot->StoH_ARM2_Joint_Status[1] == 1)
    {
        ui->arm2_flag2->setText("Y");
    }
    else
    {
        ui->arm2_flag2->setText("N");
    }
    // joint 3
    if (socket_robot->StoH_ARM2_Joint_Status[2] == 1)
    {
        ui->arm2_flag3->setText("Y");
    }
    else
    {
        ui->arm2_flag3->setText("N");
    }
    // joint 4
    if (socket_robot->StoH_ARM2_Joint_Status[3] == 1)
    {
        ui->arm2_flag4->setText("Y");
    }
    else
    {
        ui->arm2_flag4->setText("N");
    }
    // joint 5
    if (socket_robot->StoH_ARM2_Joint_Status[4] == 1)
    {
        ui->arm2_flag5->setText("Y");
    }
    else
    {
        ui->arm2_flag5->setText("N");
    }
    // joint 6
    if (socket_robot->StoH_ARM2_Joint_Status[5] == 1)
    {
        ui->arm2_flag6->setText("Y");
    }
    else
    {
        ui->arm2_flag6->setText("N");
    }
}

void MainWindow::update_vehicle_speed()
{
    double speed = socket_robot->StoH_Vehicle_Speed;
    stringstream ss;
    ss << speed << "m/s";
    string tmp;
    ss >> tmp;
    ui->label_speed_value->setText(QString(tmp.c_str()));
}

void MainWindow::update_arm1_actuator_posture()
{
    char buffer[8];

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM1_Actuator_Posture[0]);
    ui->arm1_x->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM1_Actuator_Posture[1]);
    ui->arm1_y->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM1_Actuator_Posture[2]);
    ui->arm1_z->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM1_Actuator_Posture[3]);
    ui->arm1_alpha->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM1_Actuator_Posture[4]);
    ui->arm1_beta->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM1_Actuator_Posture[5]);
    ui->arm1_gamma->setText(buffer);
}

void MainWindow::update_arm2_actuator_posture()
{
    char buffer[8];

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM2_Actuator_Posture[0]);
    ui->arm2_x->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM2_Actuator_Posture[1]);
    ui->arm2_y->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM2_Actuator_Posture[2]);
    ui->arm2_z->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM2_Actuator_Posture[3]);
    ui->arm2_alpha->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM2_Actuator_Posture[4]);
    ui->arm2_beta->setText(buffer);

    sprintf(buffer,"%.3f",socket_robot->StoH_ARM2_Actuator_Posture[5]);
    ui->arm2_gamma->setText(buffer);
}

void MainWindow::update_error_code()
{
    char buffer[4];

    sprintf(buffer,"%04d",socket_robot->StoH_Error_Code);
    ui->Error_Code->setText(buffer);
}


void MainWindow::update_all_data()
{
    update_arm1_joint_angle();
    update_arm1_joint_status();
    update_arm1_actuator_posture();
    update_arm2_joint_angle();
    update_arm2_joint_status();
    update_arm2_actuator_posture();
    update_vehicle_speed();
}






void MainWindow::update_label_realsense()
{
    int N=image_processor.realsense_numb();
    QImage qimg;
    QImage qimg1;
    QImage qimg2;
       switch(N)
    {
       case 1:
         // 1
        if(image_processor.rgbd_id_list[RGBD_L_ID] == 1)
        {
            img_d435_rgb = image_processor.getImageColor();
            img_d435_depth = image_processor.depth_RStoCV(global_depth);
            qimg = image_processor.toQImage(img_d435_rgb);
            ui->label_realsense_rgb->setPixmap(QPixmap::fromImage(qimg));
            ui->label_realsense_rgb->show();
        }
        else if(image_processor.rgbd_id_list[RGBD_R_ID] == 1)
        {
            img_d435_rgb1 = image_processor.getImageColor1();
            img_d435_depth1 = image_processor.depth_RStoCV(global_depth_1);
            qimg1 = image_processor.toQImage(img_d435_rgb1);
            ui->label_realsense_rgb_2->setPixmap(QPixmap::fromImage(qimg1));
            ui->label_realsense_rgb_2->show();
        }
        else if(image_processor.rgbd_id_list[RGBD_C_ID] == 1)
        {
            img_d435_rgb2 = image_processor.getImageColor2();
            img_d435_depth2 = image_processor.depth_RStoCV(global_depth_2);
            qimg2 = image_processor.toQImage(img_d435_rgb2);
            ui->label_realsense_rgb_3->setPixmap(QPixmap::fromImage(qimg2));
            ui->label_realsense_rgb_3->show();
        }

        break;

       case 2:
           // 1
           if(image_processor.rgbd_id_list[RGBD_L_ID] == 1)
           {
               img_d435_rgb = image_processor.getImageColor();
               img_d435_depth = image_processor.depth_RStoCV(global_depth);
               qimg = image_processor.toQImage(img_d435_rgb);
               ui->label_realsense_rgb->setPixmap(QPixmap::fromImage(qimg));
               ui->label_realsense_rgb->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_R_ID] == 1)
           {
               img_d435_rgb1 = image_processor.getImageColor1();
               img_d435_depth1 = image_processor.depth_RStoCV(global_depth_1);
               qimg1 = image_processor.toQImage(img_d435_rgb1);
               ui->label_realsense_rgb_2->setPixmap(QPixmap::fromImage(qimg1));
               ui->label_realsense_rgb_2->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_C_ID] == 1)
           {
               img_d435_rgb2 = image_processor.getImageColor2();
               img_d435_depth2 = image_processor.depth_RStoCV(global_depth_2);
               qimg2 = image_processor.toQImage(img_d435_rgb2);
               ui->label_realsense_rgb_3->setPixmap(QPixmap::fromImage(qimg2));
               ui->label_realsense_rgb_3->show();
           }

            // 2
           if(image_processor.rgbd_id_list[RGBD_L_ID] == 2)
           {
               img_d435_rgb = image_processor.getImageColor();
               img_d435_depth = image_processor.depth_RStoCV(global_depth);
               qimg = image_processor.toQImage(img_d435_rgb);
               ui->label_realsense_rgb->setPixmap(QPixmap::fromImage(qimg));
               ui->label_realsense_rgb->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_R_ID] == 2)
           {
               img_d435_rgb1 = image_processor.getImageColor1();
               img_d435_depth1 = image_processor.depth_RStoCV(global_depth_1);
               qimg1 = image_processor.toQImage(img_d435_rgb1);
               ui->label_realsense_rgb_2->setPixmap(QPixmap::fromImage(qimg1));
               ui->label_realsense_rgb_2->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_C_ID] == 2)
           {
               img_d435_rgb2 = image_processor.getImageColor2();
               img_d435_depth2 = image_processor.depth_RStoCV(global_depth_2);
               qimg2 = image_processor.toQImage(img_d435_rgb2);
               ui->label_realsense_rgb_3->setPixmap(QPixmap::fromImage(qimg2));
               ui->label_realsense_rgb_3->show();
           }
           break;

       case 3:
           // 1
           if(image_processor.rgbd_id_list[RGBD_L_ID] == 1)
           {
               img_d435_rgb = image_processor.getImageColor();
               img_d435_depth = image_processor.depth_RStoCV(global_depth);
               qimg = image_processor.toQImage(img_d435_rgb);
               ui->label_realsense_rgb->setPixmap(QPixmap::fromImage(qimg));
               ui->label_realsense_rgb->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_R_ID] == 1)
           {
               img_d435_rgb1 = image_processor.getImageColor1();
               img_d435_depth1 = image_processor.depth_RStoCV(global_depth_1);
               qimg1 = image_processor.toQImage(img_d435_rgb1);
               ui->label_realsense_rgb_2->setPixmap(QPixmap::fromImage(qimg1));
               ui->label_realsense_rgb_2->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_C_ID] == 1)
           {
               img_d435_rgb2 = image_processor.getImageColor2();
               img_d435_depth2 = image_processor.depth_RStoCV(global_depth_2);
               qimg2 = image_processor.toQImage(img_d435_rgb2);
               ui->label_realsense_rgb_3->setPixmap(QPixmap::fromImage(qimg2));
               ui->label_realsense_rgb_3->show();
           }

            // 2
           if(image_processor.rgbd_id_list[RGBD_L_ID] == 2)
           {
               img_d435_rgb = image_processor.getImageColor();
               img_d435_depth = image_processor.depth_RStoCV(global_depth);
               qimg = image_processor.toQImage(img_d435_rgb);
               ui->label_realsense_rgb->setPixmap(QPixmap::fromImage(qimg));
               ui->label_realsense_rgb->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_R_ID] == 2)
           {
               img_d435_rgb1 = image_processor.getImageColor1();
               img_d435_depth1 = image_processor.depth_RStoCV(global_depth_1);
               qimg1 = image_processor.toQImage(img_d435_rgb1);
               ui->label_realsense_rgb_2->setPixmap(QPixmap::fromImage(qimg1));
               ui->label_realsense_rgb_2->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_C_ID] == 2)
           {
               img_d435_rgb2 = image_processor.getImageColor2();
               img_d435_depth2 = image_processor.depth_RStoCV(global_depth_2);
               qimg2 = image_processor.toQImage(img_d435_rgb2);
               ui->label_realsense_rgb_3->setPixmap(QPixmap::fromImage(qimg2));
               ui->label_realsense_rgb_3->show();
           }
           // 3
           if(image_processor.rgbd_id_list[RGBD_L_ID] == 3)
           {
               img_d435_rgb = image_processor.getImageColor();
               img_d435_depth = image_processor.depth_RStoCV(global_depth);
               qimg = image_processor.toQImage(img_d435_rgb);
               ui->label_realsense_rgb->setPixmap(QPixmap::fromImage(qimg));
               ui->label_realsense_rgb->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_R_ID] == 3)
           {
               img_d435_rgb1 = image_processor.getImageColor1();
               img_d435_depth1 = image_processor.depth_RStoCV(global_depth_1);
               qimg1 = image_processor.toQImage(img_d435_rgb1);
               ui->label_realsense_rgb_2->setPixmap(QPixmap::fromImage(qimg1));
               ui->label_realsense_rgb_2->show();
           }
           else if(image_processor.rgbd_id_list[RGBD_C_ID] == 3)
           {
               img_d435_rgb2 = image_processor.getImageColor2();
               img_d435_depth2 = image_processor.depth_RStoCV(global_depth_2);
               qimg2 = image_processor.toQImage(img_d435_rgb2);
               ui->label_realsense_rgb_3->setPixmap(QPixmap::fromImage(qimg2));
               ui->label_realsense_rgb_3->show();
           }
           break;

       default:
           cout<<"程序界面没有采集到深度相机的图像帧"<<endl;
       }

}

void MainWindow::update_gcamera()
{
    img_gcamera = gcamera->getImage();
    QImage qimg = gcamera->toQImage(img_gcamera);
    ui->label_gcamera_show->setPixmap(QPixmap::fromImage(qimg));
    ui->label_gcamera_show->show();
}

void MainWindow::update_left_image()
{
    img_mono_left = image_shower->getLeftImage();
    //cv::resize(img_mono_left, img_mono_left, Size(), 0.5, 0.5);
    QImage qimg = image_shower->toQImage(img_mono_left);
    ui->label_left_img->setPixmap(QPixmap::fromImage(qimg));
    ui->label_left_img->show();
}

void MainWindow::update_right_image()
{
    img_mono_right = image_shower->getRightImage();
    QImage qimg = image_shower->toQImage(img_mono_right);
    ui->label_right_img->setPixmap(QPixmap::fromImage(qimg));
    ui->label_right_img->show();
}

void MainWindow::update_omni_image()
{
    img_mono_omni = image_shower->getOmniImage();
    QImage qimg = image_shower->toQImage(img_mono_omni);
    ui->label_front_img->setPixmap(QPixmap::fromImage(qimg));
    ui->label_front_img->show();
}

void MainWindow::on_left_port_index_activated(int index)
{
    cout << "indexs = " << ui->left_port_index->itemText(index).toStdString() <<  endl;
    int id = ui->left_port_index->itemText(index).toInt();
    //    image_shower.set_left_port(id);
}


vector<QString> MainWindow::get_camera_id_list()
{
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    cout << "We detect camera nums: " << cameras.size() << endl;

    vector<QString> mono_cameras_index;

    for(int i = 0; i < cameras.size(); ++i)
    {
        cout << "cameras: " << i << ": "<< cameras.at(i).description().toStdString() << endl;
        cout << "id: " << *(cameras.at(i).deviceName().toStdString().end() - 1) << endl;
        if(cameras.at(i).description().toStdString() != "Intel(R) RealSense(TM) Depth Ca")
        {
            string index = cameras.at(i).deviceName().toStdString();
            char id = *(index.end()-1);
            mono_cameras_index.push_back(QString(id));
            cout << "id: " << id << endl;
        }
    }

    return mono_cameras_index;
}

vector<float> MainWindow::get_arm1_values()
{
    vector<float> arm_values(6);
    arm_values[0] = ui->arm1_slider1->value()+180;
    arm_values[1] = ui->arm1_slider2->value()+180;
    arm_values[2] = ui->arm1_slider3->value()+180;
    arm_values[3] = ui->arm1_slider4->value()+180;
    arm_values[4] = ui->arm1_slider5->value()+180;
    arm_values[5] = ui->arm1_slider6->value()+180;
    return arm_values;
}

vector<float> MainWindow::get_arm2_values()
{
    vector<float> arm_values(6);
    arm_values[0] = ui->arm2_slider1->value()+180;
    arm_values[1] = ui->arm2_slider2->value()+180;
    arm_values[2] = ui->arm2_slider3->value()+180;
    arm_values[3] = ui->arm2_slider4->value()+180;
    arm_values[4] = ui->arm2_slider5->value()+180;
    arm_values[5] = ui->arm2_slider6->value()+180;
    return arm_values;
}


void MainWindow::show_depth(int x, int y) {
    if(!img_d435_depth.empty())
    {
        Mat depth = img_d435_depth.clone();
        double val = depth.at<ushort>(y,x)/1000.;
        stringstream ss;
        ss << val << "m";
        string tmp;
        ss >> tmp;
        ui->label_depth_value->setText(QString(tmp.c_str()));
    }
}

void MainWindow::show_depth_1(int x, int y) {
    if(!img_d435_depth1.empty())
    {
        Mat depth1 = img_d435_depth1.clone();
        double val = depth1.at<ushort>(y,x)/1000.;
        stringstream ss;
        ss << val << "m";
        string tmp;
        ss >> tmp;
        ui->label_depth_value->setText(QString(tmp.c_str()));
    }
}

void MainWindow::show_depth_2(int x, int y) {
    if(!img_d435_depth2.empty())
    {
        Mat depth2 = img_d435_depth2.clone();
        double val = depth2.at<ushort>(y,x)/1000.;
        stringstream ss;
        ss << val << "m";
        string tmp;
        ss >> tmp;
        ui->label_depth_value->setText(QString(tmp.c_str()));
    }
}


bool MainWindow::scan_port_list()
{
    ui->combo_box_serial_port->clear();
    //查找可用的串口
    int cnt = 0;
    for(auto &info: QSerialPortInfo::availablePorts())
//    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);
        qDebug() << serial.portName();
        ui->combo_box_serial_port->addItem(serial.portName());
        if(serial.open(QIODevice::ReadWrite))
        {
            ui->combo_box_serial_port->addItem(serial.portName());
            ui->combo_box_serial_port->setCurrentIndex(0);
            serial.close();
            cnt++;
        }
    }
    return (cnt == 0)?false:true;
}

void MainWindow::on_scan_port_button_clicked()
{
    scan_port_list();
}

void MainWindow::scan_realsense_list()
{
    rs2::context ctx; // Create librealsense context for managing devices

    // Start a streaming pipe per each connected device
    for (auto&& dev : ctx.query_devices())
    {
//        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
//        cout << "serial_number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << endl;
//        pipe.start(cfg);
//        pipelines.emplace_back(pipe);
    }


}

/**
 * @brief MainWindow::on_serial_connect_button_clicked
 * 这里对应串口通信类，当然你可以在主函数里边调试使用，这里是针对云台控制部分而写的代码
 */
void MainWindow::on_serial_connect_button_clicked()
{

    /*
     * 判断如果没有端口存在的话就不会进行任何操作，所以一般事先要按扫描按钮，当然你也可以直接写到初始化里边
     * ，这样就可以避免点击扫描，意思就是说程序初始化就进行扫描操作，只需要点击连接.
     * 如果把这个if注释掉的话，就可以测试连接与断开开关的操作。建议保留，这样符合常理
     */
    if(!ui->combo_box_serial_port->count())//检验有没有端口
    {
        QMessageBox::information(nullptr, "云台串口检测", "未检测到串口，请先扫描并检测连线！");
        return;
    }

    if(ui->label_485_connect->text()==tr("断开"))
    {
        //关闭串口
        serial->disconnect();
        ui->label_485_connect->setText(tr("连接"));
        ui->combo_box_serial_port->setEnabled(true);
        ui->scan_port_button->setEnabled(true);
        serial_is_ok = false;
    }
    else
    {

        QString port = ui->combo_box_serial_port->currentText();
        int rate = 2400;
        int data_bit = 8;
        int check_bit = 0;
        int stop_bit = 1;

        serial = new Serial(port, rate, data_bit, check_bit, stop_bit);

        ui->combo_box_serial_port->setEnabled(false);
        ui->scan_port_button->setEnabled(false);
        ui->label_485_connect->setText(tr("断开"));
        serial_is_ok = true;

    }


}


void MainWindow::on_robot_control_stop_clicked(bool check)
{
    if(check == true)
    {
        string msg_info = Utility::toString<int>(HtoS_Vehicle_Stop)+"1";
        cout << "标识符+长度: " << HtoS_Vehicle_Stop << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        ui->Car_Enable->setText("失能");
    }
    else
    {
        string msg_info = Utility::toString<int>(HtoS_Vehicle_Stop)+"0";
        cout << "标识符+长度: " << HtoS_Vehicle_Stop << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::send_arm1_joint_angle()
{
    arm1_curr_pose = Arm_pose(get_arm1_values());
    Updata_Arm1_Angle_Display(arm1_curr_pose);
    string msg_info = Utility::toString<int>(HtoS_ARM1_Joint_Angle)+arm1_curr_pose.toStdString();
    cout << "标识符+长度: " << HtoS_ARM1_Joint_Angle << endl;
    cout << "     data: " << arm1_curr_pose.toStdString() << endl;
    socket_robot->send_msg(msg_info);
    arm1_last_pose = arm1_curr_pose;
}

void MainWindow::send_arm2_joint_angle()
{
    arm2_curr_pose = Arm_pose(get_arm2_values());
    Updata_Arm2_Angle_Display(arm2_curr_pose);
    string msg_info = Utility::toString<int>(HtoS_ARM2_Joint_Angle)+arm2_curr_pose.toStdString();
    cout << "标识符+长度: " << HtoS_ARM2_Joint_Angle << endl;
    cout << "     data: " << arm2_curr_pose.toStdString() << endl;
    socket_robot->send_msg(msg_info);
    arm2_last_pose = arm2_curr_pose;
}

void MainWindow::Updata_Arm1_Angle_Display(Arm_pose arm1_curr_pose)
{
    ui->arm1_joint_angle_1->setText(QString::number(arm1_curr_pose.joint1 - 180,10,0));
    ui->arm1_joint_angle_2->setText(QString::number(arm1_curr_pose.joint2 - 180,10,0));
    ui->arm1_joint_angle_3->setText(QString::number(arm1_curr_pose.joint3 - 180,10,0));
    ui->arm1_joint_angle_4->setText(QString::number(arm1_curr_pose.joint4 - 180,10,0));
    ui->arm1_joint_angle_5->setText(QString::number(arm1_curr_pose.joint5 - 180,10,0));
    ui->arm1_joint_angle_6->setText(QString::number(arm1_curr_pose.joint6 - 180,10,0));
}

void MainWindow::Updata_Arm2_Angle_Display(Arm_pose arm2_curr_pose)
{
    ui->arm2_joint_angle_1->setText(QString::number(arm2_curr_pose.joint1 - 180,10,0));
    ui->arm2_joint_angle_2->setText(QString::number(arm2_curr_pose.joint2 - 180,10,0));
    ui->arm2_joint_angle_3->setText(QString::number(arm2_curr_pose.joint3 - 180,10,0));
    ui->arm2_joint_angle_4->setText(QString::number(arm2_curr_pose.joint4 - 180,10,0));
    ui->arm2_joint_angle_5->setText(QString::number(arm2_curr_pose.joint5 - 180,10,0));
    ui->arm2_joint_angle_6->setText(QString::number(arm2_curr_pose.joint6 - 180,10,0));
}

void MainWindow::on_CarSpeed_clicked()
{
    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_Vehicle_Speed;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_Vehicle_Speed<< endl;
    socket_robot->send_msg(msg_info);
}


void MainWindow::on_ARM1AngleUpdata_clicked()
{
    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_ARM1_Joint_Angle;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_ARM1_Joint_Angle<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_ARM2AngleUpdata_clicked()
{
    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_ARM2_Joint_Angle;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_ARM2_Joint_Angle<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_ARM1PostureUpdata_clicked()
{
    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_ARM1_Actuator_Posture;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_ARM1_Actuator_Posture<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Init_clicked()
{
    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_All_Data;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_All_Data<< endl;
    socket_robot->send_msg(msg_info);
}



void MainWindow::on_ARM1StatusUpdata_clicked()
{
        string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_ARM1_Joint_Status;
        cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
        cout << "     data: " << ide_StoH_ARM1_Joint_Status<< endl;
        socket_robot->send_msg(msg_info);
}

void MainWindow::on_ARM2StatusUpdata_clicked()
{
    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_ARM2_Joint_Status;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_ARM2_Joint_Status<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_ARM2PostureUpdata_clicked()
{
    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_ARM2_Actuator_Posture;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_ARM2_Actuator_Posture<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_gcamera_control_stop_clicked()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_right_pressed()
{
    serial->send_data(TURN_RIGHT);
}

void MainWindow::on_gcamera_control_right_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_up_pressed()
{
    serial->send_data(TURN_UP);
}

void MainWindow::on_gcamera_control_up_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_left_pressed()
{
    serial->send_data(TURN_LEFT);
}

void MainWindow::on_gcamera_control_left_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_down_pressed()
{
    serial->send_data(TURN_DOWN);
}

void MainWindow::on_gcamera_control_down_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_up_right_pressed()
{
    serial->send_data(TURN_UP_RIGHT);
}

void MainWindow::on_gcamera_control_up_right_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_up_left_pressed()
{
    serial->send_data(TURN_UP_LEFT);
}

void MainWindow::on_gcamera_control_up_left_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_down_left_pressed()
{
    serial->send_data(TURN_DOWN_LEFT);
}

void MainWindow::on_gcamera_control_down_left_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_gcamera_control_down_right_pressed()
{
    serial->send_data(TURN_DOWN_RIGHT);
}

void MainWindow::on_gcamera_control_down_right_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_zoom_less_pressed()
{
    serial->send_data(ZOOM_MINUS);
}

void MainWindow::on_zoom_less_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_zoom_more_pressed()
{
    serial->send_data(ZOOM_PLUS);
}

void MainWindow::on_zoom_more_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_focus_less_pressed()
{
    serial->send_data(FOCUS_MINUS);
}

void MainWindow::on_focus_less_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_focus_more_pressed()
{
    serial->send_data(FOCUS_PLUS);
}

void MainWindow::on_focus_more_released()
{
    serial->send_data(STOP);
}

void MainWindow::on_mask_open_clicked()
{
     serial->send_data(MASK_ON);
}

void MainWindow::on_mask_close_clicked()
{
    serial->send_data(MASK_OFF);
}

void MainWindow::on_aperture_open_clicked()
{
    serial->send_data(APERTURE_ON);
}

void MainWindow::on_aperture_close_clicked()
{
    serial->send_data(APERTURE_OFF);
}


void MainWindow::Arm_L_Task_released()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Forward_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"100000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "100000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Forward_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Back_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"010000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "010000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Back_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Left_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"001000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "001000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Left_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Right_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000100000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000100000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Up_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000010000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000010000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Right_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Up_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Down_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000001000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000001000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Down_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_R_Forward_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"100000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "100000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Forward_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Back_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"010000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "010000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Left_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"001000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "001000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Right_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000100000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000100000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Up_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000010000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000010000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Down_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000001000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000001000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Back_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Left_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Right_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Up_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Down_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_L_Stop_clicked(bool check)
{
    on_Arm_Stop(check);
}

void MainWindow::on_Arm_R_task_released()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Stop_clicked(bool check)
{
    on_Arm_Stop(check);
}

void MainWindow::on_tabWidget_3_tabBarClicked(int index)
{
    if(index == 0  || index == 1 || index == 2)
    {
        string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"1";
        cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
        cout << "     data: " << "1"<< endl;
        socket_robot->send_msg(msg_info);
    }
    else if(index == 3 || index == 4)
    {
        string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"0";
        cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
        cout << "     data: " << "0"<< endl;
        socket_robot->send_msg(msg_info);
    }

    switch(index)
    {
    case 0:
    {
        ui->label_Arm_Joint_or_Task->setText("关节空间");//关节空间 任务空间
        ui->label_Arm_L_or_R->setText("右左机械臂");//右机械臂 左机械臂
        cout << "右左机械臂关节空间" << endl;
        break;
    }
    case 1:
    {
        ui->label_Arm_Joint_or_Task->setText("关节空间");//关节空间 任务空间
        ui->label_Arm_L_or_R->setText("左机械臂");//右机械臂 左机械臂
        cout << "左机械臂关节空间" << endl;
        break;
    }
    case 2:
    {
        ui->label_Arm_Joint_or_Task->setText("关节空间");//关节空间 任务空间
        ui->label_Arm_L_or_R->setText("右机械臂");//右机械臂 左机械臂
        cout << "右机械臂关节空间" << endl;
        break;
    }
    case 3:
    {
        ui->label_Arm_Joint_or_Task->setText("任务空间");//关节空间 任务空间
        ui->label_Arm_L_or_R->setText("左机械臂");//右机械臂 左机械臂
        cout << "左机械臂任务空间" << endl;

        if(ui->label_Arm_Point_Continue->text() == "连续模式")
        {
            ui->Arm_L_Back->setDisabled(1);
            ui->Arm_L_Down->setDisabled(1);
            ui->Arm_L_Forward->setDisabled(1);
            ui->Arm_L_Left->setDisabled(1);
            ui->Arm_L_Right->setDisabled(1);
            ui->Arm_L_Up->setDisabled(1);

            ui->Arm_L_X_nega->setDisabled(1);
            ui->Arm_L_X_posi->setDisabled(1);
            ui->Arm_L_Y_nega->setDisabled(1);
            ui->Arm_L_Y_posi->setDisabled(1);
            ui->Arm_L_Z_nega->setDisabled(1);
            ui->Arm_L_Z_posi->setDisabled(1);
        }
        else
        {
            ui->Arm_L_Back->setEnabled(1);
            ui->Arm_L_Down->setEnabled(1);
            ui->Arm_L_Forward->setEnabled(1);
            ui->Arm_L_Left->setEnabled(1);
            ui->Arm_L_Right->setEnabled(1);
            ui->Arm_L_Up->setEnabled(1);

            ui->Arm_L_X_nega->setEnabled(1);
            ui->Arm_L_X_posi->setEnabled(1);
            ui->Arm_L_Y_nega->setEnabled(1);
            ui->Arm_L_Y_posi->setEnabled(1);
            ui->Arm_L_Z_nega->setEnabled(1);
            ui->Arm_L_Z_posi->setEnabled(1);
        }

        break;
    }
    case 4:
    {
        ui->label_Arm_Joint_or_Task->setText("任务空间");//关节空间 任务空间
        ui->label_Arm_L_or_R->setText("右机械臂");//右机械臂 左机械臂
        cout << "右机械臂任务空间" << endl;

        if(ui->label_Arm_Point_Continue->text() == "连续模式")
        {
            ui->Arm_R_Back->setDisabled(1);
            ui->Arm_R_Down->setDisabled(1);
            ui->Arm_R_Forward->setDisabled(1);
            ui->Arm_R_Left->setDisabled(1);
            ui->Arm_R_Right->setDisabled(1);
            ui->Arm_R_Up->setDisabled(1);

            ui->Arm_R_X_nega->setDisabled(1);
            ui->Arm_R_X_posi->setDisabled(1);
            ui->Arm_R_Y_nega->setDisabled(1);
            ui->Arm_R_Y_posi->setDisabled(1);
            ui->Arm_R_Z_nega->setDisabled(1);
            ui->Arm_R_Z_posi->setDisabled(1);
        }
        else
        {
            ui->Arm_R_Back->setEnabled(1);
            ui->Arm_R_Down->setEnabled(1);
            ui->Arm_R_Forward->setEnabled(1);
            ui->Arm_R_Left->setEnabled(1);
            ui->Arm_R_Right->setEnabled(1);
            ui->Arm_R_Up->setEnabled(1);

            ui->Arm_R_X_nega->setEnabled(1);
            ui->Arm_R_X_posi->setEnabled(1);
            ui->Arm_R_Y_nega->setEnabled(1);
            ui->Arm_R_Y_posi->setEnabled(1);
            ui->Arm_R_Z_nega->setEnabled(1);
            ui->Arm_R_Z_posi->setEnabled(1);
        }

        break;
    }
    case 5:
    {
        if(ui->label_Arm_Point_Continue->text() == "连续模式")
        {
            ui->Left_Arm_Position_1->setDisabled(1);
            ui->Left_Arm_Position_2->setDisabled(1);
            ui->Left_Arm_Position_3->setDisabled(1);

            ui->Right_Arm_Position_1->setDisabled(1);
            ui->Right_Arm_Position_2->setDisabled(1);
            ui->Right_Arm_Position_3->setDisabled(1);
            ui->Right_Arm_Position_4->setDisabled(1);
            ui->Right_Arm_Position_5->setDisabled(1);

            ui->Both_Arm_Position_1->setDisabled(1);
            ui->Both_Arm_Position_2->setDisabled(1);
            ui->Both_Arm_Position_3->setDisabled(1);
            ui->Both_Arm_Position_4->setDisabled(1);
        }
        else
        {
            ui->Left_Arm_Position_1->setEnabled(1);
            ui->Left_Arm_Position_2->setEnabled(1);
            ui->Left_Arm_Position_3->setEnabled(1);

            ui->Right_Arm_Position_1->setEnabled(1);
            ui->Right_Arm_Position_2->setEnabled(1);
            ui->Right_Arm_Position_3->setEnabled(1);
            ui->Right_Arm_Position_4->setEnabled(1);
            ui->Right_Arm_Position_5->setEnabled(1);

            ui->Both_Arm_Position_1->setEnabled(1);
            ui->Both_Arm_Position_2->setEnabled(1);
            ui->Both_Arm_Position_3->setEnabled(1);
            ui->Both_Arm_Position_4->setEnabled(1);
        }

        break;
    }

    }
}

void MainWindow::on_robot_control_up_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"10000";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "10000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_down_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"01000";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "01000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_left_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00100";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00100" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_right_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00010";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00010" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_up_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_down_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_left_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_right_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}


void MainWindow::on_ARM1Enable_clicked(bool checked)
{
    if(ui->ARM1Enable->text() == "失能")
    {
        ui->ARM1Enable->setText("使能");
        ui->ARM1Enable_2->setText("使能");
        ui->ARM1Enable_3->setText("使能");

        string msg_info = Utility::toString<int>(HtoS_ARM1_Enable)+"1";
        cout << "标识符+长度: " << HtoS_ARM1_Enable << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        ui->ARM1Enable->setText("失能");
        ui->ARM1Enable_2->setText("失能");
        ui->ARM1Enable_3->setText("失能");

        string msg_info = Utility::toString<int>(HtoS_ARM1_Enable)+"0";
        cout << "标识符+长度: " << HtoS_ARM1_Enable << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_ARM2Enable_clicked(bool checked)
{
    if(ui->ARM2Enable->text() == "失能")
    {
        ui->ARM2Enable->setText("使能");
        ui->ARM2Enable_2->setText("使能");
        ui->ARM2Enable_3->setText("使能");

        string msg_info = Utility::toString<int>(HtoS_ARM2_Enable)+"1";
        cout << "标识符+长度: " << HtoS_ARM2_Enable << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        ui->ARM2Enable->setText("失能");
        ui->ARM2Enable_2->setText("失能");
        ui->ARM2Enable_3->setText("失能");
        string msg_info = Utility::toString<int>(HtoS_ARM2_Enable)+"0";
        cout << "标识符+长度: " << HtoS_ARM2_Enable << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_OneKeyCan_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_OneKey_Can)+"1";
    cout << "标识符+长度: " << HtoS_OneKey_Can << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_Stop_All_clicked(bool check)
{
    if(check == true)
    {
        string msg_info = Utility::toString<int>(HtoS_All_Stop)+"1";
        cout << "标识符+长度: " << HtoS_All_Stop << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        ui->ARM1Enable->setText("失能");
        ui->ARM1Enable_2->setText("失能");
        ui->ARM1Enable_3->setText("失能");

        ui->ARM2Enable->setText("失能");
        ui->ARM2Enable_2->setText("失能");
        ui->ARM2Enable_3->setText("失能");

        ui->Car_Enable->setText("失能");


    }
    else
    {
        string msg_info = Utility::toString<int>(HtoS_All_Stop)+"0";
        cout << "标识符+长度: " << HtoS_All_Stop << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }

}

void MainWindow::on_Arm_Stop_All_2_clicked(bool check)
{
    on_Arm_Stop_All_clicked(check);
}



void MainWindow::on_Jog_Step_sliderReleased()
{
    float Jog_Step = ui->Jog_Step->value();
    string str_Jog_Step = Utility::paddingZero(Jog_Step);

    string msg_info = Utility::toString<int>(HtoS_Arm_Jog_Step) + str_Jog_Step;
    cout << "标识符+长度: " << HtoS_Arm_Jog_Step << endl;
    cout << "     data: " << str_Jog_Step << endl;
    socket_robot->send_msg(msg_info);

    ui->Jog_Step_2->setValue(Jog_Step);
    ui->Jog_Step_3->setValue(Jog_Step);
    ui->Jog_Step_4->setValue(Jog_Step);
}

void MainWindow::on_Can_Open_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Can_Open) + "1";
    cout << "标识符+长度: " << HtoS_Can_Open << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);

}

void MainWindow::on_Can_Open_released()
{
    string msg_info = Utility::toString<int>(HtoS_Can_Open) + "0";
    cout << "标识符+长度: " << HtoS_Can_Open << endl;
    cout << "     data: " << "0" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Can_Close_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Can_Close) + "1";
    cout << "标识符+长度: " << HtoS_Can_Close << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Can_Close_released()
{
    string msg_info = Utility::toString<int>(HtoS_Can_Close) + "0";
    cout << "标识符+长度: " << HtoS_Can_Close << endl;
    cout << "     data: " << "0" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Left_Arm_Position_1_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmLeft_Position) + "0001.000";
    cout << "标识符+长度: " << HtoS_ArmLeft_Position << endl;
    cout << "     data: " << "0001.000" << endl;
    socket_robot->send_msg(msg_info);

}

void MainWindow::on_Left_Arm_Position_2_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmLeft_Position) + "0002.000";
    cout << "标识符+长度: " << HtoS_ArmLeft_Position << endl;
    cout << "     data: " << "0002.000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Left_Arm_Position_3_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmLeft_Position) + "0003.000";
    cout << "标识符+长度: " << HtoS_ArmLeft_Position << endl;
    cout << "     data: " << "0003.000" << endl;
    socket_robot->send_msg(msg_info);
}


void MainWindow::on_Right_Arm_Position_1_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmRight_Position) + "0001.000";
    cout << "标识符+长度: " << HtoS_ArmRight_Position << endl;
    cout << "     data: " << "0001.000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Right_Arm_Position_2_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmRight_Position) + "0002.000";
    cout << "标识符+长度: " << HtoS_ArmRight_Position << endl;
    cout << "     data: " << "0002.000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Right_Arm_Position_3_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmRight_Position) + "0003.000";
    cout << "标识符+长度: " << HtoS_ArmRight_Position << endl;
    cout << "     data: " << "0003.000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Hand_Open_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_Hand_Open) + "1";
    cout << "标识符+长度: " << HtoS_Hand_Open << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}


void MainWindow::on_Hand_Close_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Hand_Close) + "1";
    cout << "标识符+长度: " << HtoS_Hand_Close << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Hand_Close_released()
{
    string msg_info = Utility::toString<int>(HtoS_Hand_Close) + "0";
    cout << "标识符+长度: " << HtoS_Hand_Close << endl;
    cout << "     data: " << "0" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Light_clicked(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Light) + "1";
        cout << "标识符+长度: " << HtoS_Light << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        ui->Light->setText("照明关");
    }
    else
    {
        string msg_info = Utility::toString<int>(HtoS_Light) + "0";
        cout << "标识符+长度: " << HtoS_Light << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);

        ui->Light->setText("照明开");
    }
}

void MainWindow::on_Error_Ask_clicked()
{

    string msg_info = Utility::toString<int>(Both_Host_Ask_Data)+ide_StoH_Error_Code;
    cout << "标识符+长度: " << Both_Host_Ask_Data << endl;
    cout << "     data: " << ide_StoH_Error_Code << endl;
    socket_robot->send_msg(msg_info);

}

void MainWindow::on_robot_control_up_right_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"10010";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "10010" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_up_left_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"10100";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "10100" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_down_right_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"01010";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "01010" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_down_left_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"01100";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "01100" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_up_right_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_up_left_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_down_right_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_robot_control_down_left_released()
{
    string msg_info = Utility::toString<int>(HtoS_Vehicle_Status)+"00001";
    cout << "标识符+长度: " << HtoS_Vehicle_Status << endl;
    cout << "     data: " << "00001" << endl;
    socket_robot->send_msg(msg_info);
}


void MainWindow::on_Car_Enable_clicked(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Vehicle_Enable) + "1";
        cout << "标识符+长度: " << HtoS_Vehicle_Enable << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        ui->Car_Enable->setText("使能");
    }
    else
    {
        string msg_info = Utility::toString<int>(HtoS_Vehicle_Enable) + "0";
        cout << "标识符+长度: " << HtoS_Vehicle_Enable << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);

        ui->Car_Enable->setText("失能");
    }
}

void MainWindow::on_Arm_Tool_1_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0001.000";
        cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
        cout << "     data: " << "0001.000" << endl;
        socket_robot->send_msg(msg_info);
        ui->Tool_Show->setText("刀具1");
        ui->flywheel_Out->setEnabled(0);
        ui->flywheel_In->setEnabled(0);
    }
}

void MainWindow::on_Arm_Tool_2_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0002.000";
        cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
        cout << "     data: " << "0002.000" << endl;
        socket_robot->send_msg(msg_info);
        ui->Tool_Show->setText("刀具2");
        ui->flywheel_Out->setEnabled(0);
        ui->flywheel_In->setEnabled(0);
    }
}

void MainWindow::on_Arm_Tool_3_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0003.000";
        cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
        cout << "     data: " << "0003.000" << endl;
        socket_robot->send_msg(msg_info);
        ui->Tool_Show->setText("刀具3");
        ui->flywheel_Out->setEnabled(0);
        ui->flywheel_In->setEnabled(0);
    }
}

void MainWindow::on_Arm_Tool_4_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0004.000";
        cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
        cout << "     data: " << "0004.000" << endl;
        socket_robot->send_msg(msg_info);
        ui->Tool_Show->setText("刀具4");
        ui->flywheel_Out->setEnabled(0);
        ui->flywheel_In->setEnabled(0);
    }
}


void MainWindow::handleTimeout_askdata()
{
    if(Flag_Timeout == 1)
    {
        // 请求错误代码
        on_Error_Ask_clicked();
        Flag_Timeout ++;
    }
    else if(Flag_Timeout == 2)
    {
        // 请求右机械臂位姿
        on_ARM1PostureUpdata_clicked();
        Flag_Timeout ++;
    }
    else if(Flag_Timeout == 3)
    {
        // 请求左机械臂位姿
        on_ARM2PostureUpdata_clicked();
        Flag_Timeout ++;
    }
    else if(Flag_Timeout == 4)
    {
        // 请求车体速度
        on_CarSpeed_clicked();
        Flag_Timeout ++;
    }
    else if(Flag_Timeout == 5)
    {
        // 请求右机械臂状态
        on_ARM1StatusUpdata_clicked();
        Flag_Timeout ++;
    }
    else if(Flag_Timeout == 6)
    {
        // 请求左机械臂状态
        on_ARM2StatusUpdata_clicked();
        Flag_Timeout ++;
    }
    else if(Flag_Timeout == 7)
    {
        // 请求右机械臂角度
        on_ARM1AngleUpdata_clicked();
        Flag_Timeout ++;
    }
    else if(Flag_Timeout == 8)
    {
        // 请求左机械臂角度
        on_ARM2AngleUpdata_clicked();
        Flag_Timeout ++;

    }
    else
    {
        Flag_Timeout = 1;
    }

}


void MainWindow::on_OneKeyBalance_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_OneKey_Balance)+"1";
    cout << "标识符+长度: " << HtoS_OneKey_Balance << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_OneKeyDisposal_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_OneKey_Disposal)+"1";
    cout << "标识符+长度: " << HtoS_OneKey_Disposal << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Climb_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Intelligent_Function)+"1";
        cout << "标识符+长度: " << HtoS_Intelligent_Function << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_Surmount_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Intelligent_Function)+"2";
        cout << "标识符+长度: " << HtoS_Intelligent_Function << endl;
        cout << "     data: " << "2" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_Stair_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Intelligent_Function)+"3";
        cout << "标识符+长度: " << HtoS_Intelligent_Function << endl;
        cout << "     data: " << "3" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_Uneven_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Intelligent_Function)+"4";
        cout << "标识符+长度: " << HtoS_Intelligent_Function << endl;
        cout << "     data: " << "4" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_button_close_all_cameras_clicked()
{

    cout << "stop rgbd" << endl;
    image_processor.stop();     // close rgbd camera

    cout << "stop mono" << endl;
    image_shower->stop();       // close mono camera

    cout << "stop global" << endl;
    gcamera->stop();            // close global camera

    watchdog->start_flag = false;

    ui->button_launch_camera->setEnabled(1);

    ui->button_close_all_cameras->setDisabled(1);

}

void MainWindow::on_flag_rgbd_3D_1_clicked()
{
    show3D.setCamera_num(1);
    show3D.init();
}

void MainWindow::on_flag_rgbd_3D_2_clicked()
{
    show3D_1.setCamera_num(2);
    show3D_1.init();
}

void MainWindow::on_flag_rgbd_3D_3_clicked()
{
    show3D_2.setCamera_num(3);
    show3D_2.init();
}

void MainWindow::serial_terminal_ReadData()
{
    bool flag_finish = false;

    if(serial_terminal->read_data(terminal.data))
    {
        QByteArray bs = terminal.data.toLatin1();
        flag_finish = terminal.check_frame_start_end();
        if(flag_finish)
        {
            if( terminal.data_save() )
            {
                terminal_To_ui();

            }
            terminal.data.clear();
        }
    }


}

void MainWindow::arm_point_or_continue()
{
    if(terminal.cmd_point_or_continue != terminal.cmd_point_or_continue_old)
    {
        if(terminal.cmd_point_or_continue == PRESS)
        {
            string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue) + '1';
            cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
            cout << "     data: " << '1' << endl;
            socket_robot->send_msg(msg_info);

            ui->ARM_Point_Continue->setText("连续");
            ui->ARM_Point_Continue_2->setText("连续");
            ui->ARM_Point_Continue_3->setText("连续");
            ui->ARM_Point_Continue_4->setText("连续");
            ui->label_Arm_Point_Continue->setText("连续模式");
        }
        else if(terminal.cmd_point_or_continue == RELEASE)
        {
            string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue) + '0';
            cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
            cout << "     data: " << '0' << endl;
            socket_robot->send_msg(msg_info);

            ui->ARM_Point_Continue->setText("点动");
            ui->ARM_Point_Continue_2->setText("点动");
            ui->ARM_Point_Continue_3->setText("点动");
            ui->ARM_Point_Continue_4->setText("点动");
            ui->label_Arm_Point_Continue->setText("点动模式");
        }
    }
}

void MainWindow::change_tool_enable()
{
    if(terminal.change_tool_enable != terminal.change_tool_enable_old)
    {
        if(terminal.change_tool_enable == PRESS)
        {
            switch(terminal.tool_num)
            {
             case 0:
             {
                string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0000.000";
                cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
                cout << "     data: " << "0000.000" << endl;
                socket_robot->send_msg(msg_info);
                ui->Tool_Show->setText("刀具0");
                break;
             }
            case 1:
            {
                string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0001.000";
                cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
                cout << "     data: " << "0001.000" << endl;
                socket_robot->send_msg(msg_info);
                ui->Tool_Show->setText("刀具1");
                break;
            }
            case 2:
            {
                string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0002.000";
                cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
                cout << "     data: " << "0002.000" << endl;
                socket_robot->send_msg(msg_info);
                ui->Tool_Show->setText("刀具2");
                break;
            }
            case 3:
            {
                string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0003.000";
                cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
                cout << "     data: " << "0003.000" << endl;
                socket_robot->send_msg(msg_info);
                ui->Tool_Show->setText("刀具3");
                break;
            }
            case 4:
            {
                string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0004.000";
                cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
                cout << "     data: " << "0004.000" << endl;
                socket_robot->send_msg(msg_info);
                ui->Tool_Show->setText("刀具4");
                break;
            }
            }

        }
        else if(terminal.change_tool_enable == RELEASE)
        {


        }
        else
        {
            cout << "error_focus_more" << endl;
        }
    }
}

void MainWindow::ptz_focus_less()
{
    if(terminal.cmd_ptz_focus_less != terminal.cmd_ptz_focus_less_old)
    {
        if(terminal.cmd_ptz_focus_less == PRESS)
        {
            on_focus_less_pressed();
        }
        else if(terminal.cmd_ptz_focus_less == RELEASE)
        {
            on_focus_less_released();
        }
        else
        {
            cout << "error_focus_less" << endl;
        }
    }
}

void MainWindow::ptz_zoom_more()
{
    if(terminal.cmd_ptz_zoom_more != terminal.cmd_ptz_zoom_more_old)
    {
        if(terminal.cmd_ptz_zoom_more == PRESS)
        {
            on_zoom_more_pressed();
        }
        else if(terminal.cmd_ptz_zoom_more == RELEASE)
        {
            on_zoom_more_released();
        }
        else
        {
            cout << "error_zoom_more" << endl;
        }
    }
}

void MainWindow::ptz_zoom_less()
{
    if(terminal.cmd_ptz_zoom_less != terminal.cmd_ptz_zoom_less_old)
    {
        if(terminal.cmd_ptz_zoom_less == PRESS)
        {
            on_zoom_less_pressed();
        }
        else if(terminal.cmd_ptz_zoom_less == RELEASE)
        {
            on_zoom_less_released();
        }
        else
        {
            cout << "error_zoom_less" << endl;
        }
    }
}

void MainWindow::launch_camera()
{
    if(terminal.cmd_launch_camera != terminal.cmd_launch_camera_old)
    {
        if(terminal.cmd_launch_camera == PRESS)
        {
            on_button_launch_camera_clicked();
        }
        else if(terminal.cmd_launch_camera == RELEASE)
        {
            on_button_close_all_cameras_clicked();
        }
        else
        {
             cout << "error_launch_camera" << endl;
        }
    }
}

void MainWindow::light()
{
    if(terminal.cmd_light != terminal.cmd_light_old)
    {
        if(terminal.cmd_light == PRESS)
        {
            on_Light_clicked(true);

        }
        else if(terminal.cmd_light == RELEASE)
        {
            on_Light_clicked(false);
            if(terminal.flag_cmd_8910_change == true)
            {
                terminal_cmd8910();
                terminal.flag_cmd_8910_change = false;
            }
        }
        else
        {
             cout << "error_light" << endl;
        }
    }
}

void MainWindow::terminal_cmd1()
{
    arm_point_or_continue();
    change_tool_enable();
    ptz_focus_less();
    ptz_zoom_more();
    ptz_zoom_less();
    launch_camera();
    light();
}

void MainWindow::vehicle_enable()
{
    if(terminal.cmd_vehicle_enable != terminal.cmd_vehicle_enable_old)
    {
        if(terminal.cmd_vehicle_enable == PRESS)
        {
            on_Car_Enable_clicked(true);
        }
        else if(terminal.cmd_vehicle_enable == RELEASE)
        {
            on_Car_Enable_clicked(false);
        }
        else
        {
             cout << "error_vehicle_enable" << endl;
        }
    }
}

void MainWindow::left_arm_enable()
{
    if(terminal.cmd_left_arm_enable != terminal.cmd_left_arm_enable_old)
    {
        if(terminal.cmd_left_arm_enable == PRESS)
        {
            on_ARM2Enable_clicked(true);
        }
        else if(terminal.cmd_left_arm_enable == RELEASE)
        {
            on_ARM2Enable_clicked(false);
        }
        else
        {
             cout << "error_left_arm_enable" << endl;
        }
    }
}

void MainWindow::right_arm_enable()
{
    if(terminal.cmd_right_arm_enable != terminal.cmd_right_arm_enable_old)
    {
        if(terminal.cmd_right_arm_enable == PRESS)
        {
            on_ARM1Enable_clicked(true);
        }
        else if(terminal.cmd_right_arm_enable == RELEASE)
        {
            on_ARM1Enable_clicked(false);
        }
        else
        {
             cout << "error_right_arm_enable" << endl;
        }
    }
}

void MainWindow::hand_open()
{
    if(terminal.cmd_hand_open != terminal.cmd_hand_open_old)
    {
        if(terminal.cmd_hand_open == PRESS)
        {
            on_Hand_Open_clicked();
        }
        else if(terminal.cmd_hand_open == RELEASE)
        {

        }
        else
        {
            cout << "error_hand_open" << endl;
        }
    }
}

void MainWindow::hand_close()
{
    if(terminal.cmd_hand_close != terminal.cmd_hand_close_old)
    {
        if(terminal.cmd_hand_close == PRESS)
        {
            on_Hand_Close_pressed();
        }
        else if(terminal.cmd_hand_close == RELEASE)
        {
            on_Hand_Close_released();
        }
        else
        {
            cout << "error_hand_close" << endl;
        }
    }
}

void MainWindow::arms_init_pose()
{
    if(terminal.cmd_arms_init_pose != terminal.cmd_arms_init_pose_old)
    {
        if(terminal.cmd_arms_init_pose == PRESS)
        {
            string msg_info = Utility::toString<int>(HtoS_Arms_Pos_Init) + '1';
            cout << "标识符+长度: " << HtoS_Arms_Pos_Init << endl;
            cout << "     data: " << '1' << endl;
            socket_robot->send_msg(msg_info);
        }
        else if(terminal.cmd_arms_init_pose == RELEASE)
        {
        }
        else
        {
            cout << "error_arms_init_pose" << endl;
        }
    }
}

void MainWindow::arms_end_pose()
{
    if(terminal.cmd_arms_end_pose != terminal.cmd_arms_end_pose_old)
    {
        if(terminal.cmd_arms_end_pose == PRESS)
        {
            string msg_info = Utility::toString<int>(HtoS_Arms_Pos_End) + '1';
            cout << "标识符+长度: " << HtoS_Arms_Pos_End << endl;
            cout << "     data: " << '1' << endl;
            socket_robot->send_msg(msg_info);
        }
        else if(terminal.cmd_arms_end_pose == RELEASE)
        {

        }
        else
        {
            cout << "error_arms_end_pose" << endl;
        }
    }
}

void MainWindow::terminal_cmd2()
{
    vehicle_enable();
    left_arm_enable();
    right_arm_enable();
    hand_open();
    hand_close();
    arms_init_pose();
    arms_end_pose();
        //BIT7 undefine
}

void MainWindow::onekey_can()
{
    if(terminal.cmd_onekey_can != terminal.cmd_onekey_can_old)
    {
//        flag_vert = true;

        if(terminal.cmd_onekey_can == PRESS)
        {
//            flag_vert = false;
            on_OneKeyCan_clicked();


        }
        else if(terminal.cmd_onekey_can == RELEASE)
        {
//            flag_vert = true;
        }
        else
        {
            cout << "error_onekey_can" << endl;
        }
    }
}

void MainWindow::onekey_balance()
{
    if(terminal.cmd_onekey_balance != terminal.cmd_onekey_balance_old)
    {
        if(terminal.cmd_onekey_balance == PRESS)
        {
            on_OneKeyBalance_clicked();
        }
        else if(terminal.cmd_onekey_balance == RELEASE)
        {

        }
        else
        {
            cout << "error_onekey_balance" << endl;
        }
    }
}

void MainWindow::onekey_disposal()
{
    if(terminal.cmd_onekey_disposal != terminal.cmd_onekey_disposal_old)
    {
        if(terminal.cmd_onekey_disposal == PRESS)
        {
            on_OneKeyDisposal_clicked();
        }
        else if(terminal.cmd_onekey_disposal == RELEASE)
        {

        }
        else
        {
            cout << "error_onekey_disposal" << endl;
        }
    }
}

void MainWindow::yuntai_left_move()
{
    if(terminal.cmd_yuntai_left_move != terminal.cmd_yuntai_left_move_old)
    {
        if(terminal.cmd_yuntai_left_move == PRESS)
        {
            on_gcamera_control_left_pressed();
        }
        else if(terminal.cmd_yuntai_left_move == RELEASE)
        {
            on_gcamera_control_left_released();
        }
        else
        {
            cout << "error_yuntai_left_move" << endl;
        }
    }
}

void MainWindow::yuntai_right_move()
{
    if(terminal.cmd_yuntai_right_move != terminal.cmd_yuntai_right_move_old)
    {
        if(terminal.cmd_yuntai_right_move == PRESS)
        {
            on_gcamera_control_right_pressed();
        }
        else if(terminal.cmd_yuntai_right_move == RELEASE)
        {
            on_gcamera_control_right_released();
        }
        else
        {
            cout << "error_yuntai_right_move" << endl;
        }
    }
}
void MainWindow::setArmTips()
{
    if(terminal.cmd_arm1_arm2_change == RELEASE && terminal.cmd_joint_space_change == RELEASE)
    {
        string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"0";
        cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
        cout << "     data: " << "0"<< endl;
        socket_robot->send_msg(msg_info);

        ui->tabWidget_3->setCurrentIndex(3);
        ui->label_Arm_L_or_R->setText("左机械臂");
        ui->label_Arm_Joint_or_Task->setText("任务空间");
    }
    else if(terminal.cmd_arm1_arm2_change == RELEASE && terminal.cmd_joint_space_change == PRESS)
    {
        string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"1";
        cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
        cout << "     data: " << "1"<< endl;
        socket_robot->send_msg(msg_info);

        ui->tabWidget_3->setCurrentIndex(1);
        ui->label_Arm_L_or_R->setText("左机械臂");
        ui->label_Arm_Joint_or_Task->setText("关节空间");
    }
    else if(terminal.cmd_arm1_arm2_change == PRESS && terminal.cmd_joint_space_change == RELEASE)
    {
        string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"0";
        cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
        cout << "     data: " << "0"<< endl;
        socket_robot->send_msg(msg_info);

        ui->tabWidget_3->setCurrentIndex(4);
        ui->label_Arm_L_or_R->setText("右机械臂");
        ui->label_Arm_Joint_or_Task->setText("任务空间");
    }
    else if(terminal.cmd_arm1_arm2_change == PRESS && terminal.cmd_joint_space_change == PRESS)
    {
        string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"1";
        cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
        cout << "     data: " << "1"<< endl;
        socket_robot->send_msg(msg_info);

        ui->tabWidget_3->setCurrentIndex(2);
        ui->label_Arm_L_or_R->setText("右机械臂");
        ui->label_Arm_Joint_or_Task->setText("关节空间");
    }

/*
    clock_t delay;  //定义clock_t类型的变量，表示延时时间
    double sec = 1;
    delay = sec * CLOCKS_PER_SEC;   //delay赋值为secs 乘以 CLOCKS_PER_SEC值，将输入的秒数转化系统的时间
    clock_t start=clock();    //定义clock_t类型变量start，并赋值为当前系统的时间
    while(clock()-start < delay);  // 如果当前时间减去上一刻的系统时间小于延时的系统时间，则执行循环等待，否则跳出循


    if(terminal.cmd_point_or_continue == PRESS)
    {
        ui->ARM_Point_Continue->setText("连续");
        ui->ARM_Point_Continue_2->setText("连续");
        ui->ARM_Point_Continue_3->setText("连续");
        ui->ARM_Point_Continue_4->setText("连续");
        ui->label_Arm_Point_Continue->setText("连续模式");

        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue) + '1';
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << '1' << endl;
        socket_robot->send_msg(msg_info);

    }
    else if(terminal.cmd_point_or_continue == RELEASE)
    {
        ui->ARM_Point_Continue->setText("点动");
        ui->ARM_Point_Continue_2->setText("点动");
        ui->ARM_Point_Continue_3->setText("点动");
        ui->ARM_Point_Continue_4->setText("点动");
        ui->label_Arm_Point_Continue->setText("点动模式");

        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue) + '0';
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << '0' << endl;
        socket_robot->send_msg(msg_info);

    }
*/



}

void MainWindow::arm1_arm2_change()
{
    if(terminal.cmd_arm1_arm2_change != terminal.cmd_arm1_arm2_change_old)
    {
        setArmTips();
    }
}

void MainWindow::yuntai_up_move()
{
    if(terminal.cmd_yuntai_up_move != terminal.cmd_yuntai_up_move_old)
    {
        if(terminal.cmd_yuntai_up_move == PRESS)
        {
            on_gcamera_control_up_pressed();
        }
        else if(terminal.cmd_yuntai_up_move == RELEASE)
        {
            on_gcamera_control_up_released();
        }
        else
        {
            cout << "error_yuntai_up_move" << endl;
        }
    }
}

void MainWindow::terminal_cmd3()
{
    onekey_can();
    onekey_balance();
    onekey_disposal();
    yuntai_left_move();
    yuntai_right_move();
    arm1_arm2_change();
    yuntai_up_move();
}

void MainWindow::robot_control_stop()
{
    if(terminal.cmd_robot_control_stop != terminal.cmd_robot_control_stop_old)
    {
        if(terminal.cmd_robot_control_stop == PRESS)
        {
           on_robot_control_stop_clicked(true);
        }
        else if(terminal.cmd_robot_control_stop == RELEASE)
        {
            on_robot_control_stop_clicked(false);
        }
        else
        {
            cout << "error_robot_control_stop" << endl;
        }
    }
}

void MainWindow::Arm_L_Stop()
{
    if(terminal.cmd_left_arm_stop != terminal.cmd_left_arm_stop_old)
    {
        if(terminal.cmd_left_arm_stop == PRESS)
        {
            string msg_info = Utility::toString<int>(HtoS_Arm_StopAll) + '1';
            cout << "标识符+长度: " << HtoS_Arm_StopAll << endl;
            cout << "     data: " << '1' << endl;
            socket_robot->send_msg(msg_info);
        }
        else if(terminal.cmd_left_arm_stop == RELEASE)
        {
            string msg_info = Utility::toString<int>(HtoS_Arm_StopAll) + '0';
            cout << "标识符+长度: " << HtoS_Arm_StopAll << endl;
            cout << "     data: " << '0' << endl;
            socket_robot->send_msg(msg_info);
        }
        else
        {
            cout << "error_left_arm_stop" << endl;
        }
    }
}

void MainWindow::Arm_R_Stop()
{
    if(terminal.cmd_right_arm_stop != terminal.cmd_right_arm_stop_old)
    {
        if(terminal.cmd_right_arm_stop == PRESS)
        {
            string msg_info = Utility::toString<int>(HtoS_Arm_StopAll) + '1';
            cout << "标识符+长度: " << HtoS_Arm_StopAll << endl;
            cout << "     data: " << '1' << endl;
            socket_robot->send_msg(msg_info);
        }
        else if(terminal.cmd_right_arm_stop == RELEASE)
        {
            string msg_info = Utility::toString<int>(HtoS_Arm_StopAll) + '0';
            cout << "标识符+长度: " << HtoS_Arm_StopAll << endl;
            cout << "     data: " << '0' << endl;
            socket_robot->send_msg(msg_info);
        }
        else
        {
            cout << "error_right_arm_stop" << endl;
        }
    }

}

void MainWindow::Stop_All()
{
    if(terminal.cmd_all_stop != terminal.cmd_all_stop_old)
    {
        if(terminal.cmd_all_stop == PRESS)
        {

            on_Arm_Stop_All_clicked(true);

        }
        else if(terminal.cmd_all_stop == RELEASE)
        {
            on_Arm_Stop_All_clicked(false);
        }
        else
        {
            cout << "error_all_arm_stop" << endl;
        }
    }
}


void MainWindow::send_ARMRight_Taskspace_Statue()
{
    if(terminal.cmd_point_or_continue == RELEASE && ui->label_Arm_Point_Continue->text() == "点动模式")
    {
        string data = terminal.get_arm_joint_space_status();
        string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue) + data;
        cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
        cout << "     data: " << data << endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        cout << "请切换点动连续模式" << endl;
    }

}

void MainWindow::send_ARMLeft_Taskspace_Statue()
{
    string data = terminal.get_arm_joint_space_status();
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue) + data;
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << data << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::send_ARMRight_Jointspace_Statue()
{
    string data = terminal.get_arm_joint_space_status();
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue) + data;
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << data << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::send_ARMLeft_Jointspace_Statue()
{
    string data = terminal.get_arm_joint_space_status();
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue) + data;
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << data << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::send_ARM_Statue()
{

    if(terminal.cmd_joint_space_change == '0') //task space
    {
        if(terminal.cmd_arm1_arm2_change == '0' )//left arm
        {
            if(terminal.cmd_left_arm_stop == '0'
                    && terminal.cmd_all_stop == '0')
                send_ARMLeft_Taskspace_Statue();
        }
        else//right arm
        {
            if(terminal.cmd_right_arm_stop == '0'
                    && terminal.cmd_all_stop == '0')
                send_ARMRight_Taskspace_Statue();
        }
    }
    else //joint space
    {
        if(terminal.cmd_arm1_arm2_change == '0')//left arm
        {
            if(terminal.cmd_left_arm_stop == '0'
                    && terminal.cmd_all_stop == '0')
                send_ARMLeft_Jointspace_Statue();
        }
        else//right arm
        {
            if(terminal.cmd_right_arm_stop == '0'
                    && terminal.cmd_all_stop == '0')
                send_ARMRight_Jointspace_Statue();
        }
    }
}

void MainWindow::Arm_Forward_1_True()
{
    if(terminal.cmd_arm_forward_1_true != terminal.cmd_arm_forward_1_true_old)
    {
        if(terminal.cmd_arm_forward_1_true == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_Left_2_True()
{
    if(terminal.cmd_arm_left_2_true != terminal.cmd_arm_left_2_true_old)
    {
        if(terminal.cmd_arm_left_2_true == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_Up_3_True()
{
    if(terminal.cmd_arm_up_3_true != terminal.cmd_arm_up_3_true_old)
    {
        if(terminal.cmd_arm_up_3_true == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_X_True_4_True()
{
    if(terminal.cmd_arm_x_true_4_true != terminal.cmd_arm_x_true_4_true_old)
    {
        if(terminal.cmd_arm_x_true_4_true == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}


void MainWindow::terminal_cmd4()
{
    robot_control_stop();
    Arm_L_Stop();
    Arm_R_Stop();
    Stop_All();

    Arm_Forward_1_True();
    Arm_Left_2_True();
    Arm_Up_3_True();
    Arm_X_True_4_True();
}

void MainWindow::Arm_Y_True_5_True()
{
    if(terminal.cmd_arm_y_true_5_true != terminal.cmd_arm_y_true_5_true_old)
    {
        if(terminal.cmd_arm_y_true_5_true == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_Z_True_6_True()
{
    if(terminal.cmd_arm_z_true_6_true != terminal.cmd_arm_z_true_6_true_old)
    {
        if(terminal.cmd_arm_z_true_6_true == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_Back_1_False()
{
    if(terminal.cmd_arm_back_1_false != terminal.cmd_arm_back_1_false_old)
    {
        if(terminal.cmd_arm_back_1_false == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_Right_2_False()
{
    if(terminal.cmd_arm_right_2_false != terminal.cmd_arm_right_2_false_old)
    {
        if(terminal.cmd_arm_right_2_false == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }

}

void MainWindow::Arm_Down_3_False()
{
    if(terminal.cmd_arm_down_3_false != terminal.cmd_arm_down_3_false_old)
    {
        if(terminal.cmd_arm_down_3_false == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_X_False_4_False()
{
    if(terminal.cmd_arm_x_false_4_false != terminal.cmd_arm_x_false_4_false_old)
    {
        if(terminal.cmd_arm_x_false_4_false == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_Y_False_5_False()
{
    if(terminal.cmd_arm_y_false_5_false != terminal.cmd_arm_y_false_5_false_old)
    {
        if(terminal.cmd_arm_y_false_5_false == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}

void MainWindow::Arm_Z_False_6_False()
{
    if(terminal.cmd_arm_z_false_6_false != terminal.cmd_arm_z_false_6_false_old)
    {
        if(terminal.cmd_arm_z_false_6_false == '0')
        {
            terminal.flag_arm_bool_move_stop = true;
            terminal.sent_arm_bool_cnt = 0;
        }
        send_ARM_Statue();
    }
}


void MainWindow::terminal_cmd5()
{
    Arm_Y_True_5_True();
    Arm_Z_True_6_True();
    Arm_Back_1_False();
    Arm_Right_2_False();
    Arm_Down_3_False();
    Arm_X_False_4_False();
    Arm_Y_False_5_False();
    Arm_Z_False_6_False();
}

void MainWindow::climb()
{
//    if(terminal.cmd_climb != terminal.cmd_climb_old)
//    {
//        if(terminal.cmd_climb == PRESS)
//        {
//            on_Climb_toggled(true);
//        }
//        else if(terminal.cmd_climb == RELEASE)
//        {

//        }
//        else
//        {
//            cout << "error_climb" << endl;
//        }
//    }
}

void MainWindow::arm_tool_4()
{
    if(terminal.cmd_arm_tool_4 != terminal.cmd_arm_tool_4_old)
    {
        if(terminal.cmd_arm_tool_4 == PRESS)
        {
            terminal.tool_num = 4;

        }
        else if(terminal.cmd_arm_tool_4 == RELEASE)
        {

        }
        else
        {
            cout << "error_arm_tool_4" << endl;
        }
    }
}

void MainWindow::surmount()
{
//    if(terminal.cmd_surmount != terminal.cmd_surmount_old)
//    {
//        if(terminal.cmd_surmount == PRESS)
//        {
//            on_Surmount_toggled(true);
//        }
//        else if(terminal.cmd_surmount == RELEASE)
//        {

//        }
//        else
//        {
//            cout << "error_surmount" << endl;
//        }
//    }
}

void MainWindow::arm_tool_3()
{
    if(terminal.cmd_arm_tool_3 != terminal.cmd_arm_tool_3_old)
    {
        if(terminal.cmd_arm_tool_3 == PRESS)
        {
            terminal.tool_num = 3;

        }

        else if(terminal.cmd_arm_tool_3 == RELEASE)
        {

        }
        else
        {
            cout << "error_arm_tool_3" << endl;
        }
    }
}

void MainWindow::stair()
{
//    if(terminal.cmd_stair != terminal.cmd_stair_old)
//    {
//        if(terminal.cmd_stair == PRESS)
//        {
//            on_Stair_toggled(true);
//        }
//        else if(terminal.cmd_stair == RELEASE)
//        {

//        }
//        else
//        {
//            cout << "error_stair" << endl;
//        }
//    }
}

void MainWindow::arm_tool_2()
{
    if(terminal.cmd_arm_tool_2 != terminal.cmd_arm_tool_2_old)
    {
        if(terminal.cmd_arm_tool_2 == PRESS)
        {
            terminal.tool_num = 2;

        }

        else if(terminal.cmd_arm_tool_2 == RELEASE)
        {

        }
        else
        {
            cout << "error_arm_tool_2" << endl;
        }
    }
}

void MainWindow::uneven()
{
//    if(terminal.cmd_uneven != terminal.cmd_uneven_old)
//    {
//        if(terminal.cmd_uneven == PRESS)
//        {
//            on_Uneven_toggled(true);
//        }
//        else if(terminal.cmd_uneven == RELEASE)
//        {

//        }
//        else
//        {
//            cout << "error_uneven" << endl;
//        }
//    }
}

void MainWindow::arm_tool_1()
{
    if(terminal.cmd_arm_tool_1 != terminal.cmd_arm_tool_1_old)
    {
        if(terminal.cmd_arm_tool_1 == PRESS)
        {
            terminal.tool_num = 1;

        }

        else if(terminal.cmd_arm_tool_1 == RELEASE)
        {

        }
        else
        {
            cout << "error_arm_tool_1" << endl;
        }
    }
}

void MainWindow::arm_reset()
{
    if(terminal.cmd_Arm_Reset != terminal.cmd_Arm_Reset_old)
    {
        if(terminal.cmd_Arm_Reset == PRESS)
        {
            string msg_info = Utility::toString<int>(HtoS_ARM_Reset) + "1";
            cout << "标识符+长度: " << HtoS_ARM_Reset << endl;
            cout << "     data: " << "1" << endl;
            socket_robot->send_msg(msg_info);
        }
    }
}

void MainWindow::yuntai_down_move()
{
    if(terminal.cmd_yuntai_down_move != terminal.cmd_yuntai_down_move_old)
    {
        if(terminal.cmd_yuntai_down_move == PRESS)
        {
            on_gcamera_control_down_pressed();
        }
        else if(terminal.cmd_yuntai_down_move == RELEASE)
        {
            on_gcamera_control_down_released();
        }
        else
        {
            cout << "error_yuntai_down_move" << endl;
        }
    }
}

void MainWindow::Choose_Workspace()
{
    if(terminal.cmd_joint_space_change != terminal.cmd_joint_space_change_old)
    {
        if(terminal.cmd_joint_space_change == PRESS)
        {
//            string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"1";
//            cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
//            cout << "     data: " << "1"<< endl;
//            socket_robot->send_msg(msg_info);

            setArmTips();

        }
        else if(terminal.cmd_joint_space_change == RELEASE)
        {
//            string msg_info = Utility::toString<int>(HtoS_Choose_Workspace)+"0";
//            cout << "标识符+长度: " << HtoS_Choose_Workspace << endl;
//            cout << "     data: " << "0"<< endl;
//            socket_robot->send_msg(msg_info);

            setArmTips();
        }
        else
        {
            cout << "error_joint_space_change" << endl;
        }
    }
}

void MainWindow::terminal_cmd6()
{
    arm_tool_4(); //climb();
    arm_tool_3(); //surmount();
    arm_tool_2(); //stair();
    arm_tool_1(); //uneven();
    arm_reset();
    yuntai_down_move();
    Choose_Workspace();
}


void MainWindow::terminal_cmd7()
{


}



void MainWindow::terminal_cmd8910()
{
    bool flag_change = terminal.if_roll2_change();
    if( flag_change )
    {
       double a = -terminal.cmd_roll2_x + 1.0;
       double b = terminal.cmd_roll2_y + 1.0;
       double c = -terminal.cmd_roll2_z + 1.0;

       if(a < 0.0)    a = 0.0;    else if(a > 2.0) a = 2.0;
       if(b < 0.0)    b = 0.0;    else if(b > 2.0) b = 2.0;
       if(c < 0.0)    c = 0.0;    else if(c > 2.0) c = 2.0;


       if(((a-1)*(a-1)+(b-1)*(b-1))<ROLL_Limit && terminal.cmd_roll2_button == PRESS)
           terminal.flag_right_vert = true;
       else if(((a-1)*(a-1)+(b-1)*(b-1))<ROLL_Limit && terminal.cmd_roll2_button == RELEASE)
           terminal.flag_right_vert = false;

       if(terminal.flag_right_vert == false)
       {
               string data = Utility::paddingZero(b)
                            + Utility::paddingZero(a)
                            + Utility::paddingZero(c)
                            +Utility::paddingZero(1.0)
                            +Utility::paddingZero(1.0)
                            +Utility::paddingZero(1.0);
               string msg_info = Utility::toString<int>(HtoS_RightArm_data) + data;
               cout << "标识符+长度: " << HtoS_RightArm_data << endl;
               cout << "     data: " << data << endl;
               socket_robot->send_msg(msg_info);
       }
       else
       {
               string data = Utility::paddingZero(1.0)
                            +Utility::paddingZero(1.0)
                            +Utility::paddingZero(1.0)
                            +Utility::paddingZero(b)
                            + Utility::paddingZero(a)
                            + Utility::paddingZero(c);

               string msg_info = Utility::toString<int>(HtoS_RightArm_data) + data;
               cout << "标识符+长度: " << HtoS_RightArm_data << endl;
               cout << "     data: " << data << endl;
               socket_robot->send_msg(msg_info);
       }

    }


}

void MainWindow::terminal_cmd111213()
{
    bool flag_change = terminal.if_vehicle_stick_change();
    if( flag_change )
    {
       double a = terminal.cmd_vehicle_theta + 180.0;
       double b = terminal.cmd_vehicle_pho;
       double c = terminal.cmd_vehicle_self_rot + 1.0;

       if(a < 0.0)    a = 0.0;    else if(a > 360.0) a = 360.0;
       if(b < 0.0)    b = 0.0;    else if(b > 1.0) b = 1.0;
       if(c < 0.0)    c = 0.0;    else if(c > 2.0) c = 2.0;

       string data = Utility::paddingZero(a)
                      + Utility::paddingZero(b)
                      + Utility::paddingZero(c);
       string msg_info = Utility::toString<int>(HtoS_Vehicle_stick) + data;
       cout << "标识符+长度: " << HtoS_Vehicle_stick << endl;
       cout << "     data: " << data << endl;
       socket_robot->send_msg(msg_info);
    }
}

void MainWindow::terminal_cmd141516()
{
    bool flag_change = terminal.if_roll1_change();
    if( flag_change )
    {
       double a = -terminal.cmd_roll1_x + 1.0;
       double b = terminal.cmd_roll1_y + 1.0;
       double c = -terminal.cmd_roll1_z + 1.0;

       if(a < 0.0)    a = 0.0;    else if(a > 2.0) a = 2.0;
       if(b < 0.0)    b = 0.0;    else if(b > 2.0) b = 2.0;
       if(c < 0.0)    c = 0.0;    else if(c > 2.0) c = 2.0;

       if(((a-1)*(a-1)+(b-1)*(b-1))<ROLL_Limit && terminal.cmd_roll1_button == PRESS)
           terminal.flag_left_vert = true;
       else if(((a-1)*(a-1)+(b-1)*(b-1))<ROLL_Limit && terminal.cmd_roll1_button == RELEASE)
           terminal.flag_left_vert = false;

       if (terminal.flag_left_vert == false)
       {
           string data = Utility::paddingZero(b)
                        + Utility::paddingZero(a)
                        + Utility::paddingZero(c)
                        +Utility::paddingZero(1.0)
                        +Utility::paddingZero(1.0)
                        +Utility::paddingZero(1.0);
           string msg_info = Utility::toString<int>(HtoS_LeftArm_data) + data;
           cout << "标识符+长度: " << HtoS_LeftArm_data << endl;
           cout << "     data: " << data << endl;
           socket_robot->send_msg(msg_info);
       }
       else
       {
           string data = Utility::paddingZero(1.0)
                        +Utility::paddingZero(1.0)
                        +Utility::paddingZero(1.0)
                        +Utility::paddingZero(b)
                        + Utility::paddingZero(a)
                        + Utility::paddingZero(c);

           string msg_info = Utility::toString<int>(HtoS_LeftArm_data) + data;
           cout << "标识符+长度: " << HtoS_LeftArm_data << endl;
           cout << "     data: " << data << endl;
           socket_robot->send_msg(msg_info);
       }


    }
}


void MainWindow::terminal_To_ui()
{
    if(terminal.flag_cmd_1_change == true)
    {
        terminal_cmd1();
        terminal.flag_cmd_1_change = false;
    }

    if(terminal.flag_cmd_2_change == true)
    {
        terminal_cmd2();
        terminal.flag_cmd_2_change = false;
    }

    if(terminal.flag_cmd_3_change == true)
    {
        terminal_cmd3();
        terminal.flag_cmd_3_change = false;
    }
    if(terminal.flag_cmd_4_change == true)
    {
         terminal_cmd4();
         terminal.flag_cmd_4_change = false;
    }
    if(terminal.flag_cmd_5_change == true)
    {
        terminal_cmd5();
        terminal.flag_cmd_5_change = false;
    }
    if(terminal.flag_cmd_6_change == true)
    {
       terminal_cmd6();
       terminal.flag_cmd_6_change = false;
    }
    if(terminal.flag_cmd_7_change == true)
    {
        terminal_cmd7();
        terminal.flag_cmd_7_change = false;
    }
    if(terminal.flag_cmd_8910_change == true)
    {
        terminal_cmd8910();
        terminal.flag_cmd_8910_change = false;
    }
    if(terminal.flag_cmd_111213_change == true)
    {
        terminal_cmd111213();
        terminal.flag_cmd_111213_change = false;
    }
    if(terminal.flag_cmd_141516_change == true)
    {
        terminal_cmd141516();
        terminal.flag_cmd_141516_change = false;
    }

    if(terminal.flag_arm_bool_move_stop == true)
    {
        send_ARM_Statue();
        terminal.sent_arm_bool_cnt++;
        if(terminal.sent_arm_bool_cnt >= SENT_ARM_BOOL_CNT_MAX)
        {
            terminal.sent_arm_bool_cnt = 0;
            terminal.flag_arm_bool_move_stop = false;
        }
    }
}



void MainWindow::on_ARM2Enable_2_clicked(bool checked)
{
    if(ui->ARM2Enable_2->text() == "失能")
    {
        ui->ARM2Enable->setText("使能");
        ui->ARM2Enable_2->setText("使能");
        ui->ARM2Enable_3->setText("使能");

        string msg_info = Utility::toString<int>(HtoS_ARM2_Enable)+"1";
        cout << "标识符+长度: " << HtoS_ARM2_Enable << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        ui->ARM2Enable->setText("失能");
        ui->ARM2Enable_2->setText("失能");
        ui->ARM2Enable_3->setText("失能");
        string msg_info = Utility::toString<int>(HtoS_ARM2_Enable)+"0";
        cout << "标识符+长度: " << HtoS_ARM2_Enable << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_ARM1Enable_2_clicked(bool checked)
{
    if(ui->ARM1Enable_2->text() == "失能")
    {
        ui->ARM1Enable->setText("使能");
        ui->ARM1Enable_2->setText("使能");
        ui->ARM1Enable_3->setText("使能");

        string msg_info = Utility::toString<int>(HtoS_ARM1_Enable)+"1";
        cout << "标识符+长度: " << HtoS_ARM1_Enable << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        ui->ARM1Enable->setText("失能");
        ui->ARM1Enable_2->setText("失能");
        ui->ARM1Enable_3->setText("失能");

        string msg_info = Utility::toString<int>(HtoS_ARM1_Enable)+"0";
        cout << "标识符+长度: " << HtoS_ARM1_Enable << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }
}



void MainWindow::on_Arm_L_X_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000000100000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000000100000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_X_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000000010000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000000010000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Y_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000000001000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000000001000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Y_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000000000100";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000000000100"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Z_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000000000010";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000000000010"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_Z_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Taskspace_Statue)+"000000000001";
    cout << "标识符+长度: " << HtoS_ARMLeft_Taskspace_Statue << endl;
    cout << "     data: " << "000000000001"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_X_posi_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_X_nega_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Y_posi_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Y_nega_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Z_posi_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_L_Z_nega_released()
{
    Arm_L_Task_released();
}

void MainWindow::on_Arm_Stop_All_4_clicked(bool check)
{
    on_Arm_Stop_All_clicked(check);
}

void MainWindow::on_Arm_R_X_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000000100000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000000100000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_X_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000000010000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000000010000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Y_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000000001000";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000000001000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Y_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000000000100";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000000000100"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Z_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000000000010";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000000000010"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_Z_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Taskspace_Statue)+"000000000001";
    cout << "标识符+长度: " << HtoS_ARMRight_Taskspace_Statue << endl;
    cout << "     data: " << "000000000001"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_X_posi_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_X_nega_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Y_posi_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Y_nega_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Z_posi_released()
{
   on_Arm_R_task_released();
}

void MainWindow::on_Arm_R_Z_nega_released()
{
    on_Arm_R_task_released();
}

void MainWindow::on_Jog_Step_2_sliderReleased()
{
    float Jog_Step = ui->Jog_Step_2->value();
    string str_Jog_Step = Utility::paddingZero(Jog_Step);

    string msg_info = Utility::toString<int>(HtoS_Arm_Jog_Step) + str_Jog_Step;
    cout << "标识符+长度: " << HtoS_Arm_Jog_Step << endl;
    cout << "     data: " << str_Jog_Step << endl;
    socket_robot->send_msg(msg_info);

    ui->Jog_Step->setValue(Jog_Step);
    ui->Jog_Step_3->setValue(Jog_Step);
    ui->Jog_Step_4->setValue(Jog_Step);
}

void MainWindow::on_Arm_Stop(bool check)
{
    if(check == true)
    {
        string msg_info = Utility::toString<int>(HtoS_Arm_StopAll)+"1";
        cout << "标识符+长度: " << HtoS_Arm_StopAll << endl;
        cout << "     data: " << "1"<< endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        string msg_info = Utility::toString<int>(HtoS_Arm_StopAll)+"0";
        cout << "标识符+长度: " << HtoS_Arm_StopAll << endl;
        cout << "     data: " << "0"<< endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_Arm_L_Joint_Stop_clicked(bool check)
{
    on_Arm_Stop(check);

}

void MainWindow::on_Arm_Stop_All_5_clicked(bool check)
{
        on_Arm_Stop_All_clicked(check);
}

void MainWindow::on_Arm_L_1_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"100000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "100000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_1_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"010000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "010000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_2_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"001000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "001000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_2_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000100000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000100000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_3_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000010000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000010000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_3_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000001000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000001000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_4_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000000100000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000000100000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_4_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000000010000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000000010000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_5_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000000001000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000000001000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_5_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000000000100";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000000000100"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_6_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000000000010";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000000000010"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_6_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000000000001";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000000000001"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::Arm_L_Joint_released()
{
    string msg_info = Utility::toString<int>(HtoS_ARMLeft_Jointspace_Statue)+"000000000000";
    cout << "标识符+长度: " << HtoS_ARMLeft_Jointspace_Statue << endl;
    cout << "     data: " << "000000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_L_1_posi_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_1_nega_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_2_posi_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_2_nega_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_3_posi_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_3_nega_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_4_posi_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_4_nega_released()
{
     Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_5_posi_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_5_nega_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_6_posi_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_Arm_L_6_nega_released()
{
    Arm_L_Joint_released();
}

void MainWindow::on_ARM2Enable_3_clicked(bool checked)
{
    if(ui->ARM2Enable_3->text() == "失能")
    {
        ui->ARM2Enable->setText("使能");
        ui->ARM2Enable_2->setText("使能");
        ui->ARM2Enable_3->setText("使能");

        string msg_info = Utility::toString<int>(HtoS_ARM2_Enable)+"1";
        cout << "标识符+长度: " << HtoS_ARM2_Enable << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        ui->ARM2Enable->setText("失能");
        ui->ARM2Enable_2->setText("失能");
        ui->ARM2Enable_3->setText("失能");
        string msg_info = Utility::toString<int>(HtoS_ARM2_Enable)+"0";
        cout << "标识符+长度: " << HtoS_ARM2_Enable << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }

}

void MainWindow::on_Arm_R_Joint_Stop_clicked(bool check)
{
    on_Arm_Stop(check);
}

void MainWindow::on_Arm_Stop_All_6_clicked(bool check)
{
     on_Arm_Stop_All_clicked(check);
}

void MainWindow::on_Arm_R_1_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"100000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "100000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_1_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"010000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "010000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_2_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"001000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "001000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_2_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000100000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000100000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_3_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000010000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000010000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_3_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000001000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000001000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_4_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000000100000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000000100000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_4_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000000010000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000000010000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_5_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000000001000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000000001000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_5_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000000000100";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000000000100"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_6_posi_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000000000010";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000000000010"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_6_nega_pressed()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000000000001";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000000000001"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::Arm_R_Joint_released()
{
    string msg_info = Utility::toString<int>(HtoS_ARMRight_Jointspace_Statue)+"000000000000";
    cout << "标识符+长度: " << HtoS_ARMRight_Jointspace_Statue << endl;
    cout << "     data: " << "000000000000"<< endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Arm_R_1_posi_released()
{
    Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_1_nega_released()
{
    Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_2_posi_released()
{
    Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_2_nega_released()
{
    Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_3_posi_released()
{
    Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_3_nega_released()
{
   Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_4_posi_released()
{
    Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_4_nega_released()
{
   Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_5_posi_released()
{
   Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_5_nega_released()
{
   Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_6_posi_released()
{
  Arm_R_Joint_released();
}

void MainWindow::on_Arm_R_6_nega_released()
{
  Arm_R_Joint_released();
}

void MainWindow::on_ARM1Enable_3_clicked(bool checked)
{
    if(ui->ARM1Enable_3->text() == "失能")
    {
        ui->ARM1Enable->setText("使能");
        ui->ARM1Enable_2->setText("使能");
        ui->ARM1Enable_3->setText("使能");

        string msg_info = Utility::toString<int>(HtoS_ARM1_Enable)+"1";
        cout << "标识符+长度: " << HtoS_ARM1_Enable << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);
    }
    else
    {
        ui->ARM1Enable->setText("失能");
        ui->ARM1Enable_2->setText("失能");
        ui->ARM1Enable_3->setText("失能");

        string msg_info = Utility::toString<int>(HtoS_ARM1_Enable)+"0";
        cout << "标识符+长度: " << HtoS_ARM1_Enable << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);
    }
}

void MainWindow::on_Jog_Step_3_sliderReleased()
{
    float Jog_Step = ui->Jog_Step_3->value();
    string str_Jog_Step = Utility::paddingZero(Jog_Step);

    string msg_info = Utility::toString<int>(HtoS_Arm_Jog_Step) + str_Jog_Step;
    cout << "标识符+长度: " << HtoS_Arm_Jog_Step << endl;
    cout << "     data: " << str_Jog_Step << endl;
    socket_robot->send_msg(msg_info);

    ui->Jog_Step->setValue(Jog_Step);
    ui->Jog_Step_2->setValue(Jog_Step);
    ui->Jog_Step_4->setValue(Jog_Step);
}

void MainWindow::on_Jog_Step_4_sliderReleased()
{
    float Jog_Step = ui->Jog_Step_4->value();
    string str_Jog_Step = Utility::paddingZero(Jog_Step);

    string msg_info = Utility::toString<int>(HtoS_Arm_Jog_Step) + str_Jog_Step;
    cout << "标识符+长度: " << HtoS_Arm_Jog_Step << endl;
    cout << "     data: " << str_Jog_Step << endl;
    socket_robot->send_msg(msg_info);

    ui->Jog_Step->setValue(Jog_Step);
    ui->Jog_Step_2->setValue(Jog_Step);
    ui->Jog_Step_3->setValue(Jog_Step);
}

void MainWindow::on_ARM_Reset_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ARM_Reset) + "1";
    cout << "标识符+长度: " << HtoS_ARM_Reset << endl;
    cout << "     data: " << "1" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_ARM_Point_Continue_clicked()
{
    if(ui->ARM_Point_Continue->text() == "点动")
    {
        ui->ARM_Point_Continue->setText("连续");
        ui->ARM_Point_Continue_2->setText("连续");
        ui->ARM_Point_Continue_3->setText("连续");
        ui->ARM_Point_Continue_4->setText("连续");
        ui->label_Arm_Point_Continue->setText("连续模式");


        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"1";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        Disable_Tool_All();
    }
    else
    {
        ui->ARM_Point_Continue->setText("点动");
        ui->ARM_Point_Continue_2->setText("点动");
        ui->ARM_Point_Continue_3->setText("点动");
        ui->ARM_Point_Continue_4->setText("点动");
        ui->label_Arm_Point_Continue->setText("点动模式");

        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"0";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);

        Enable_Tool_All();
    }
}

void MainWindow::on_ARM_Point_Continue_2_clicked()
{
    if(ui->ARM_Point_Continue_2->text() == "点动")
    {
        ui->ARM_Point_Continue->setText("连续");
        ui->ARM_Point_Continue_2->setText("连续");
        ui->ARM_Point_Continue_3->setText("连续");
        ui->ARM_Point_Continue_4->setText("连续");
        ui->label_Arm_Point_Continue->setText("连续模式");


        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"1";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        Disable_Tool_All();
    }
    else
    {
        ui->ARM_Point_Continue->setText("点动");
        ui->ARM_Point_Continue_2->setText("点动");
        ui->ARM_Point_Continue_3->setText("点动");
        ui->ARM_Point_Continue_4->setText("点动");
        ui->label_Arm_Point_Continue->setText("点动模式");

        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"0";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);

        Enable_Tool_All();
    }
}



void MainWindow::on_Terminal_Face_Update_clicked()
{
    setArmTips();
}


void MainWindow::on_Car_Speed_Slide_sliderReleased()
{
    string CarSpeed_rank = Utility::paddingZero(ui->Car_Speed_Slide->value());
    string msg_info = Utility::toString<int>(HtoS_CarSpeed_rank) + CarSpeed_rank + ".000";
    cout << "标识符+长度: " << HtoS_CarSpeed_rank << endl;
    cout << "     data: " << CarSpeed_rank << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Shotdown_clicked()
{
    QMessageBox:: StandardButton result= QMessageBox::information(NULL, "Shotdown", "是否关机",QMessageBox::Yes|QMessageBox::No);
    switch (result)
    {
    case QMessageBox::Yes:
    {
        string msg_info = Utility::toString<int>(HtoS_System_Showdown)+"1";
        cout << "标识符+长度: " << HtoS_System_Showdown << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        clock_t delay;  //定义clock_t类型的变量，表示延时时间
        double sec = 2;
        delay = sec * CLOCKS_PER_SEC;   //delay赋值为secs 乘以 CLOCKS_PER_SEC值，将输入的秒数转化系统的时间
        clock_t start=clock();    //定义clock_t类型变量start，并赋值为当前系统的时间
        while(clock()-start < delay);  // 如果当前时间减去上一刻的系统时间小于延时的系统时间，则执行循环等待，否则跳出循

        system("shutdown -h now");
        break;
    }
    case QMessageBox::No:
        break;
    default:
        break;
    }
}


void MainWindow::on_ARM_Point_Continue_3_clicked()
{
    if(ui->ARM_Point_Continue_3->text() == "点动")
    {
        ui->ARM_Point_Continue->setText("连续");
        ui->ARM_Point_Continue_2->setText("连续");
        ui->ARM_Point_Continue_3->setText("连续");
        ui->ARM_Point_Continue_4->setText("连续");
        ui->label_Arm_Point_Continue->setText("连续模式");


        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"1";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        ui->Arm_L_Back->setDisabled(1);
        ui->Arm_L_Down->setDisabled(1);
        ui->Arm_L_Forward->setDisabled(1);
        ui->Arm_L_Left->setDisabled(1);
        ui->Arm_L_Right->setDisabled(1);
        ui->Arm_L_Up->setDisabled(1);

        ui->Arm_L_X_nega->setDisabled(1);
        ui->Arm_L_X_posi->setDisabled(1);
        ui->Arm_L_Y_nega->setDisabled(1);
        ui->Arm_L_Y_posi->setDisabled(1);
        ui->Arm_L_Z_nega->setDisabled(1);
        ui->Arm_L_Z_posi->setDisabled(1);

        Disable_Tool_All();
    }
    else
    {
        ui->ARM_Point_Continue->setText("点动");
        ui->ARM_Point_Continue_2->setText("点动");
        ui->ARM_Point_Continue_3->setText("点动");
        ui->ARM_Point_Continue_4->setText("点动");
        ui->label_Arm_Point_Continue->setText("点动模式");

        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"0";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);

        ui->Arm_L_Back->setEnabled(1);
        ui->Arm_L_Down->setEnabled(1);
        ui->Arm_L_Forward->setEnabled(1);
        ui->Arm_L_Left->setEnabled(1);
        ui->Arm_L_Right->setEnabled(1);
        ui->Arm_L_Up->setEnabled(1);

        ui->Arm_L_X_nega->setEnabled(1);
        ui->Arm_L_X_posi->setEnabled(1);
        ui->Arm_L_Y_nega->setEnabled(1);
        ui->Arm_L_Y_posi->setEnabled(1);
        ui->Arm_L_Z_nega->setEnabled(1);
        ui->Arm_L_Z_posi->setEnabled(1);

        Enable_Tool_All();
    }
}

void MainWindow::on_ARM_Point_Continue_4_clicked()
{

    if(ui->ARM_Point_Continue_4->text() == "点动")
    {
        ui->ARM_Point_Continue->setText("连续");
        ui->ARM_Point_Continue_2->setText("连续");
        ui->ARM_Point_Continue_3->setText("连续");
        ui->ARM_Point_Continue_4->setText("连续");
        ui->label_Arm_Point_Continue->setText("连续模式");


        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"1";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "1" << endl;
        socket_robot->send_msg(msg_info);

        ui->Arm_R_Back->setDisabled(1);
        ui->Arm_R_Down->setDisabled(1);
        ui->Arm_R_Forward->setDisabled(1);
        ui->Arm_R_Left->setDisabled(1);
        ui->Arm_R_Right->setDisabled(1);
        ui->Arm_R_Up->setDisabled(1);

        ui->Arm_R_X_nega->setDisabled(1);
        ui->Arm_R_X_posi->setDisabled(1);
        ui->Arm_R_Y_nega->setDisabled(1);
        ui->Arm_R_Y_posi->setDisabled(1);
        ui->Arm_R_Z_nega->setDisabled(1);
        ui->Arm_R_Z_posi->setDisabled(1);

        Disable_Tool_All();

    }
    else
    {
        ui->ARM_Point_Continue->setText("点动");
        ui->ARM_Point_Continue_2->setText("点动");
        ui->ARM_Point_Continue_3->setText("点动");
        ui->ARM_Point_Continue_4->setText("点动");
        ui->label_Arm_Point_Continue->setText("点动模式");

        string msg_info = Utility::toString<int>(HtoS_ARM_Point_Continue)+"0";
        cout << "标识符+长度: " << HtoS_ARM_Point_Continue << endl;
        cout << "     data: " << "0" << endl;
        socket_robot->send_msg(msg_info);

        ui->Arm_R_Back->setEnabled(1);
        ui->Arm_R_Down->setEnabled(1);
        ui->Arm_R_Forward->setEnabled(1);
        ui->Arm_R_Left->setEnabled(1);
        ui->Arm_R_Right->setEnabled(1);
        ui->Arm_R_Up->setEnabled(1);

        ui->Arm_R_X_nega->setEnabled(1);
        ui->Arm_R_X_posi->setEnabled(1);
        ui->Arm_R_Y_nega->setEnabled(1);
        ui->Arm_R_Y_posi->setEnabled(1);
        ui->Arm_R_Z_nega->setEnabled(1);
        ui->Arm_R_Z_posi->setEnabled(1);

        Enable_Tool_All();
    }
}

void MainWindow::on_gcamera_tabBarClicked(int index)
{
    switch(index)
    {
    case 0:
    {

    }
    case 1:
    {
        if(ui->label_485_connect->text()==tr("断开"))
        {
            ui->gcamera_control_down->setEnabled(1);
            ui->gcamera_control_down_left->setEnabled(1);
            ui->gcamera_control_down_right->setEnabled(1);
            ui->gcamera_control_left->setEnabled(1);
            ui->gcamera_control_right->setEnabled(1);
            ui->gcamera_control_stop->setEnabled(1);
            ui->gcamera_control_up->setEnabled(1);
            ui->gcamera_control_up_left->setEnabled(1);
            ui->gcamera_control_up_right->setEnabled(1);
            ui->zoom_less->setEnabled(1);
            ui->zoom_more->setEnabled(1);
            ui->focus_less->setEnabled(1);
            ui->focus_more->setEnabled(1);
            ui->aperture_close->setEnabled(1);
            ui->aperture_open->setEnabled(1);
            ui->mask_open->setEnabled(1);
            ui->mask_close->setEnabled(1);
        }
        else
        {
            ui->gcamera_control_down->setDisabled(1);
            ui->gcamera_control_down_left->setDisabled(1);
            ui->gcamera_control_down_right->setDisabled(1);
            ui->gcamera_control_left->setDisabled(1);
            ui->gcamera_control_right->setDisabled(1);
            ui->gcamera_control_stop->setDisabled(1);
            ui->gcamera_control_up->setDisabled(1);
            ui->gcamera_control_up_left->setDisabled(1);
            ui->gcamera_control_up_right->setDisabled(1);
            ui->zoom_less->setDisabled(1);
            ui->zoom_more->setDisabled(1);
            ui->focus_less->setDisabled(1);
            ui->focus_more->setDisabled(1);
            ui->aperture_close->setDisabled(1);
            ui->aperture_open->setDisabled(1);
            ui->mask_open->setDisabled(1);
            ui->mask_close->setDisabled(1);
        }

    }
    }
}


void MainWindow::on_scan_port_button_terminal_clicked()
{
    scan_port_list_terminal();
}

bool MainWindow::scan_port_list_terminal()
{
    ui->combo_box_serial_port_terminal->clear();
    //查找可用的串口
    int cnt = 0;
    for(auto &info: QSerialPortInfo::availablePorts())
//    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);
        qDebug() << serial.portName();
        ui->combo_box_serial_port_terminal->addItem(serial.portName());
        if(serial.open(QIODevice::ReadWrite))
        {
            ui->combo_box_serial_port_terminal->addItem(serial.portName());
            ui->combo_box_serial_port_terminal->setCurrentIndex(0);
            serial.close();
            cnt++;
        }
    }

    return (cnt == 0)?false:true;
}

void MainWindow::on_serial_connect_button_terminal_clicked()
{
    if(ui->label_232_connect->text()==tr("断开"))
    {
        //关闭串口
        serial_terminal->disconnect();
        ui->label_232_connect->setText(tr("连接"));
        ui->combo_box_serial_port_terminal->setEnabled(true);
        ui->scan_port_button_terminal->setEnabled(true);
    }
    else
    {

        QString port = ui->combo_box_serial_port_terminal->currentText();
        int rate = 115200;
        int data_bit = 8;
        int check_bit = 0;
        int stop_bit = 1;


        serial_terminal = new Serial(port, rate, data_bit, check_bit, stop_bit);
        QObject::connect(serial_terminal->serial, SIGNAL(readyRead()),this,SLOT(serial_terminal_ReadData())); //连接信号槽

        ui->combo_box_serial_port_terminal->setEnabled(false);
        ui->scan_port_button_terminal->setEnabled(false);
        ui->label_232_connect->setText(tr("断开"));

    }
}


void MainWindow::on_Both_Arm_Position_1_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_Arms_Pos_Init) + '1';
    cout << "标识符+长度: " << HtoS_Arms_Pos_Init << endl;
    cout << "     data: " << '1' << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Both_Arm_Position_2_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_Arms_Pos_End) + '1';
    cout << "标识符+长度: " << HtoS_Arms_Pos_End << endl;
    cout << "     data: " << '1' << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Both_Arm_Position_3_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_Both_Arm_Position) + "0001.000";
    cout << "标识符+长度: " << HtoS_Both_Arm_Position << endl;
    cout << "     data: " << "0001.000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Both_Arm_Position_4_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_Both_Arm_Position) + "0002.000";
    cout << "标识符+长度: " << HtoS_Both_Arm_Position << endl;
    cout << "     data: " << "0002.000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Right_Arm_Position_4_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmRight_Position) + "0004.000";
    cout << "标识符+长度: " << HtoS_ArmRight_Position << endl;
    cout << "     data: " << "0004.000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_Right_Arm_Position_5_clicked()
{
    string msg_info = Utility::toString<int>(HtoS_ArmRight_Position) + "0005.000";
    cout << "标识符+长度: " << HtoS_ArmRight_Position << endl;
    cout << "     data: " << "0005.000" << endl;
    socket_robot->send_msg(msg_info);
}


void MainWindow::on_Arm_Tool_0_toggled(bool checked)
{
    if(checked == 1)
    {
        string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0000.000";
        cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
        cout << "     data: " << "0000.000" << endl;
        socket_robot->send_msg(msg_info);
        ui->Tool_Show->setText("刀具0");

        ui->flywheel_Out->setEnabled(1);
        ui->flywheel_In->setEnabled(0);
    }
}

void MainWindow::on_gripper_force_sliderReleased()
{
    string gripper_force = Utility::paddingZero(ui->gripper_force->value());
    string msg_info = Utility::toString<int>(HtoS_Gripper_Force) + gripper_force + ".000";
    cout << "标识符+长度: " << HtoS_Gripper_Force << endl;
    cout << "     data: " << gripper_force << ".000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::on_gripper_position_sliderReleased()
{
    string gripper_position = Utility::paddingZero(ui->gripper_position->value());
    string msg_info = Utility::toString<int>(HtoS_Gripper_Position) + gripper_position + ".000";
    cout << "标识符+长度: " << HtoS_Gripper_Position << endl;
    cout << "     data: " << gripper_position << ".000" << endl;
    socket_robot->send_msg(msg_info);
}

void MainWindow::Enable_Tool_All(){
    if(ui->Tool_Show->text() == "飞轮"){
        ui->flywheel_Out->setEnabled(0);
        ui->flywheel_In->setEnabled(1);
        ui->Arm_Tool_0->setEnabled(0);
        ui->Arm_Tool_1->setEnabled(0);
        ui->Arm_Tool_2->setEnabled(0);
        ui->Arm_Tool_3->setEnabled(0);
        ui->Arm_Tool_4->setEnabled(0);
        ui->Tool_Show->setEnabled(1);
        ui->label_8->setEnabled(1);
    }
    else if (ui->Tool_Show->text() == "刀具0"){
        ui->flywheel_Out->setEnabled(1);
        ui->flywheel_In->setEnabled(0);
        ui->Arm_Tool_0->setEnabled(1);
        ui->Arm_Tool_1->setEnabled(1);
        ui->Arm_Tool_2->setEnabled(1);
        ui->Arm_Tool_3->setEnabled(1);
        ui->Arm_Tool_4->setEnabled(1);
        ui->Tool_Show->setEnabled(1);
        ui->label_8->setEnabled(1);
    }
    else{
        ui->Arm_Tool_0->setEnabled(1);
        ui->Arm_Tool_1->setEnabled(1);
        ui->Arm_Tool_2->setEnabled(1);
        ui->Arm_Tool_3->setEnabled(1);
        ui->Arm_Tool_4->setEnabled(1);
        ui->flywheel_Out->setEnabled(0);
        ui->flywheel_In->setEnabled(0);
        ui->Tool_Show->setEnabled(1);
        ui->label_8->setEnabled(1);
    }

}

void MainWindow::Disable_Tool_All(){
    ui->Arm_Tool_0->setDisabled(1);
    ui->Arm_Tool_1->setDisabled(1);
    ui->Arm_Tool_2->setDisabled(1);
    ui->Arm_Tool_3->setDisabled(1);
    ui->Arm_Tool_4->setDisabled(1);
    ui->flywheel_Out->setDisabled(1);
    ui->flywheel_In->setDisabled(1);
    ui->Tool_Show->setDisabled(1);
    ui->label_8->setDisabled(1);
}

void MainWindow::on_tabWidget_2_tabBarClicked(int index)
{
    switch(index)
    {
    case 0:
    {
        break;
    }
    case 1:
    {
        if(ui->label_Arm_Point_Continue->text() == "连续模式")
        {
            Disable_Tool_All();
        }
        else
        {
            Enable_Tool_All();
        }
        break;
    }
    case 2:
    {
        break;
    }
    }
}

void MainWindow::on_ExpShow_clicked(bool checked)
{
    if(checked)
        ui->ExpShow->setText("爆炸物隐藏");
    else
        ui->ExpShow->setText("爆炸物显示");

    image_processor.expShow = checked;
}

void MainWindow::on_flywheel_Out_clicked()
{
    if(ui->Tool_Show->text() == "刀具0"){
        ui->flywheel_Out->setEnabled(0);
        ui->flywheel_In->setEnabled(1);
        ui->Arm_Tool_0->setEnabled(0);
        ui->Arm_Tool_1->setEnabled(0);
        ui->Arm_Tool_2->setEnabled(0);
        ui->Arm_Tool_3->setEnabled(0);
        ui->Arm_Tool_4->setEnabled(0);

        string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0005.000";
        cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
        cout << "     data: " << "0005.000" << endl;
        socket_robot->send_msg(msg_info);
        ui->Tool_Show->setText("飞轮");
    }

}

void MainWindow::on_flywheel_In_clicked()
{
    if(ui->Tool_Show->text() == "飞轮"){
        ui->flywheel_Out->setEnabled(1);
        ui->flywheel_In->setEnabled(0);
        ui->Arm_Tool_0->setEnabled(1);
        ui->Arm_Tool_1->setEnabled(1);
        ui->Arm_Tool_2->setEnabled(1);
        ui->Arm_Tool_3->setEnabled(1);
        ui->Arm_Tool_4->setEnabled(1);

        string msg_info = Utility::toString<int>(HtoS_Arm_L_Tool_Choose)+"0000.000";
        cout << "标识符+长度: " << HtoS_Arm_L_Tool_Choose << endl;
        cout << "     data: " << "0000.000" << endl;
        socket_robot->send_msg(msg_info);
        ui->Tool_Show->setText("刀具0");
    }

}

void MainWindow::pingfail_handle(){
     on_Arm_Stop_All_clicked(true);
     std::cout << "ping fail" << std::endl;
}
