#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUrl>
#include <vector>
#include<ctime>
#include <QProcess>
#include"rgbd_camera/image_processor.h"
#include "mono_camera/image_shower.h"
#include "socket_robot/socket_robot.h"
#include "global_camera/global_camera.h"
#include"serial_common/serial_common.h"
#include"librealsense2/rs.hpp"
#include "dialog.h"
#include "rgbd_camera/Show3D .h"
#include "remote_terminal/remote_terminal.h"
#include "watch_dog/watch_dog.h"
#include "ping_check/ping_check.h"

//TARGET_POSE = 100248,
//ARM_CONTROL = 100348,
//CAR_CONTROL = 1004,

#define     ASK_DATA_PERIOD     500

enum MissionSend
{
    HtoS_Vehicle_Status     =       10010005,
    HtoS_Target_Posture     =       10020048,
    HtoS_ARM1_Joint_Angle   =       10030048,
    HtoS_Choose_Workspace   =       10040001,
    HtoS_ARM2_Joint_Angle   =       10050048,
    HtoS_send_naviWayPoints  =      1007,
    HtoS_ArmLeft_Workspace  =       10080006,
    HtoS_ArmRight_Workspace =       10090006,
    HtoS_CarSpeed_rank      =       10100008,
    HtoS_Arm_L_Tool_Choose  =       10110008,
    HtoS_Arm_R_Tool_Choose  =       10120008,
    HtoS_naviWayPoints_Start =      10130001,
    HtoS_ARM1_Enable        =       10140001,
    HtoS_ARM2_Enable        =       10150001,
    HtoS_OneKey_Can         =       10160001,
    HtoS_Arm_StopAll        =       10170001,
    HtoS_Arm_Jog_Step       =       10180008,
    HtoS_Can_Open           =       10190001,
    HtoS_Can_Close          =       10200001,
    HtoS_ArmLeft_Position   =       10210008,
    HtoS_ArmRight_Position  =       10220008,
    HtoS_Hand_Open          =       10230001,
    HtoS_Hand_Close         =       10240001,
    HtoS_Light              =       10250001,
    HtoS_Error_Ask          =       10260001,
    HtoS_Vehicle_Enable     =       10270001,
    HtoS_OneKey_Balance     =       10280001,
    HtoS_OneKey_Disposal    =       10290001,
    HtoS_Intelligent_Function =     10300001,
    HtoS_Vehicle_stick      =       10310024,
    HtoS_ARMRight_Taskspace_Statue      =       10320012,
    HtoS_ARMLeft_Taskspace_Statue       =       10330012,
    HtoS_ARMRight_Jointspace_Statue     =       10340012,
    HtoS_ARMLeft_Jointspace_Statue      =       10350012,
    HtoS_RightArm_data      =       10360048,
    HtoS_LeftArm_data       =       10370048,
    HtoS_ARM_Reset          =       10380001,
    HtoS_ARM_Point_Continue =       10390001,
    HtoS_Vehicle_Stop       =       10400001,
    HtoS_All_Stop           =       10410001,
    HtoS_Arms_Pos_Init      =       10420001,
    HtoS_Arms_Pos_End       =       10430001,  
    HtoS_send_graspPoints    =       1044,
    HtoS_send_autoWayPoints  =       1045,
    HtoS_graspPoints_Start   =       10460001,   
    HtoS_autoWayPoints_Start =       10470001,
    HtoS_System_Showdown     =       10480001,
    HtoS_Both_Arm_Position   =       10490008,
    HtoS_Gripper_Position    =       10500008,
    HtoS_Gripper_Force       =       10510008,
    HtoS_Flywhell_Tool       =       10520001,


    Both_Handshake          =       20010004,
    Both_Answer             =       20020004,
    Both_Host_Ask_Data      =       20030004,


};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    std::vector<QString> get_camera_id_list();
    std::vector<float> get_arm1_values();
    std::vector<float> get_arm2_values();

    QString terminal_data;
/*********************终端映射到UI Start***********************/
    // bit 终端映射到UI
        // cmd_1
    void arm_point_or_continue();
    void change_tool_enable();
    void ptz_focus_less();
    void ptz_zoom_more();
    void ptz_zoom_less();
    void launch_camera();
    void light();
        // cmd_2
    void vehicle_enable();
    void left_arm_enable();
    void right_arm_enable();
    void hand_open();
    void hand_close();
    void arms_init_pose();
    void arms_end_pose();
        // cmd_3
    void onekey_can();
    void onekey_balance();
    void onekey_disposal();
    void yuntai_left_move();
    void yuntai_right_move();
    void arm1_arm2_change();
    void yuntai_up_move();
        // cmd_4
    void robot_control_stop();
    void Arm_L_Stop();
    void Arm_R_Stop();
    void Stop_All();
    void Arm_Forward_1_True();
    void Arm_Left_2_True();
    void Arm_Up_3_True();
    void Arm_X_True_4_True();
        // cmd_5
    void Arm_Y_True_5_True();
    void Arm_Z_True_6_True();
    void Arm_Back_1_False();
    void Arm_Right_2_False();
    void Arm_Down_3_False();
    void Arm_X_False_4_False();
    void Arm_Y_False_5_False();
    void Arm_Z_False_6_False();
        // cmd_6
    void arm_tool_4();   void climb();
    void arm_tool_3();   void surmount();
    void arm_tool_2();   void stair();
    void arm_tool_1();   void uneven();
    void arm_reset();
    void yuntai_down_move();
    void Choose_Workspace();



    // byte 终端映射到UI
    void terminal_cmd1();
    void terminal_cmd2();
    void terminal_cmd3();
    void terminal_cmd4();
    void terminal_cmd5();
    void terminal_cmd6();
    void terminal_cmd7();
    void terminal_cmd8910();
    void terminal_cmd111213();
    void terminal_cmd141516();

    void send_ARMRight_Taskspace_Statue();
    void send_ARMLeft_Taskspace_Statue();
    void send_ARMRight_Jointspace_Statue();
    void send_ARMLeft_Jointspace_Statue();
    void send_ARM_Statue();

    void setArmTips();

    // 终端映射到UI
    void terminal_To_ui();    
/*********************终端映射到UI End***********************/

    bool is_set_waypoints;
    bool is_set_autowaypoints;
    bool is_set_grasppoints;
    int Flag_Timeout;



private Q_SLOTS:

    void on_button_launch_camera_clicked();

    void update_gcamera();

    void update_label_realsense();

    void update_left_image();

    void update_right_image();

    void update_omni_image();

    void update_arm1_joint_angle();

    void update_arm2_joint_angle();

    void update_arm1_joint_status();

    void update_arm2_joint_status();

    void update_vehicle_speed();

    void update_arm1_actuator_posture();

    void update_arm2_actuator_posture();

    void update_error_code();

    void update_all_data();

    void on_left_port_index_activated(int index);

    void show_depth(int x, int y);

    void show_depth_1(int x, int y);

    void show_depth_2(int x, int y);

    bool scan_port_list();

    void on_scan_port_button_clicked();

    void scan_realsense_list();

    void on_set_waypoints_clicked(bool checked);

    void send_navi_waypoints();
    void send_navi_waypoints_order();
    void send_auto_navi_waypoints();
    void send_auto_navi_waypoints_order();
    void send_grasp_positionmatrix();
    void send_grasp_positionmatrix_order();


    void on_is_estimate_ground_plane_clicked(bool checked);
    void on_set_grasppoints_clicked(bool checked);

    void on_set_autowaypoints_clicked(bool checked);

    void on_serial_connect_button_clicked();

    void on_robot_control_stop_clicked(bool check);

    void send_arm1_joint_angle();

    void send_arm2_joint_angle();

    void on_CarSpeed_clicked();

    void on_ARM1AngleUpdata_clicked();

    void on_ARM2AngleUpdata_clicked();

    void on_ARM1PostureUpdata_clicked();

    void on_Init_clicked();

    void on_ARM1StatusUpdata_clicked();

    void on_ARM2StatusUpdata_clicked();

    void on_ARM2PostureUpdata_clicked();

    void on_gcamera_control_stop_clicked();

    void on_gcamera_control_right_pressed();

    void on_gcamera_control_right_released();

    void on_gcamera_control_up_pressed();

    void on_gcamera_control_up_released();

    void on_gcamera_control_left_pressed();

    void on_gcamera_control_left_released();

    void on_gcamera_control_down_pressed();

    void on_gcamera_control_down_released();

    void on_gcamera_control_up_right_pressed();

    void on_gcamera_control_up_right_released();

    void on_gcamera_control_up_left_pressed();

    void on_gcamera_control_up_left_released();

    void on_gcamera_control_down_left_pressed();

    void on_gcamera_control_down_left_released();

    void on_gcamera_control_down_right_pressed();

    void on_gcamera_control_down_right_released();

    void on_zoom_less_pressed();

    void on_zoom_less_released();

    void on_zoom_more_pressed();

    void on_zoom_more_released();

    void on_focus_less_pressed();

    void on_focus_less_released();

    void on_focus_more_pressed();

    void on_focus_more_released();

    void on_mask_open_clicked();

    void on_mask_close_clicked();

    void on_aperture_open_clicked();

    void on_aperture_close_clicked();

    void Arm_L_Task_released();

    void on_Arm_L_Forward_pressed();

    void on_Arm_L_Forward_released();

    void on_Arm_L_Back_pressed();

    void on_Arm_L_Back_released();

    void on_Arm_L_Left_pressed();

    void on_Arm_L_Left_released();

    void on_Arm_L_Right_pressed();

    void on_Arm_L_Up_pressed();

    void on_Arm_L_Right_released();

    void on_Arm_L_Up_released();

    void on_Arm_L_Down_pressed();

    void on_Arm_L_Down_released();

    void on_Arm_R_Forward_pressed();

    void on_Arm_R_Forward_released();

    void on_Arm_R_Back_pressed();

    void on_Arm_R_Left_pressed();

    void on_Arm_R_Right_pressed();

    void on_Arm_R_Up_pressed();

    void on_Arm_R_Down_pressed();

    void on_Arm_R_Back_released();

    void on_Arm_R_Left_released();

    void on_Arm_R_Right_released();

    void on_Arm_R_Up_released();

    void on_Arm_R_Down_released();

    void on_Arm_L_Stop_clicked(bool check);

    void on_Arm_R_task_released();

    void on_Arm_R_Stop_clicked(bool check);

    void on_tabWidget_3_tabBarClicked(int index);
    
    void on_robot_control_up_pressed();

    void on_robot_control_down_pressed();

    void on_robot_control_left_pressed();

    void on_robot_control_right_pressed();

    void on_robot_control_up_released();

    void on_robot_control_down_released();

    void on_robot_control_left_released();

    void on_robot_control_right_released();

    void Updata_Arm1_Angle_Display(Arm_pose arm1_curr_pose);

    void Updata_Arm2_Angle_Display(Arm_pose arm2_curr_pose);

    void on_ARM1Enable_clicked(bool checked);

    void on_ARM2Enable_clicked(bool checked);

    void on_OneKeyCan_clicked();

    void on_Arm_Stop_All_clicked(bool check);

    void on_Arm_Stop_All_2_clicked(bool check);

    void on_Jog_Step_sliderReleased();

    void on_Can_Open_pressed();

    void on_Can_Open_released();

    void on_Can_Close_pressed();

    void on_Can_Close_released();

    void on_Left_Arm_Position_1_clicked();

    void on_Left_Arm_Position_2_clicked();

    void on_Left_Arm_Position_3_clicked();

    void on_Right_Arm_Position_1_clicked();

    void on_Right_Arm_Position_2_clicked();

    void on_Right_Arm_Position_3_clicked();

    void on_Hand_Open_clicked();

    void on_Hand_Close_pressed();

    void on_Hand_Close_released();

    void on_Light_clicked(bool checked);

    void on_Error_Ask_clicked();

    void on_robot_control_up_right_pressed();

    void on_robot_control_up_left_pressed();

    void on_robot_control_down_right_pressed();

    void on_robot_control_down_left_pressed();

    void on_robot_control_up_right_released();

    void on_robot_control_up_left_released();

    void on_robot_control_down_right_released();

    void on_robot_control_down_left_released();

    void on_Car_Enable_clicked(bool checked);

    void on_Arm_Tool_1_toggled(bool checked);

    void on_Arm_Tool_2_toggled(bool checked);

    void on_Arm_Tool_3_toggled(bool checked);

    void on_Arm_Tool_4_toggled(bool checked);

    void handleTimeout_askdata();

    void on_OneKeyBalance_clicked();

    void on_OneKeyDisposal_clicked();

    void on_Climb_toggled(bool checked);

    void on_Surmount_toggled(bool checked);

    void on_Stair_toggled(bool checked);

    void on_Uneven_toggled(bool checked);

    void on_button_close_all_cameras_clicked();

    void on_flag_rgbd_3D_1_clicked();

    void on_flag_rgbd_3D_2_clicked();

    void on_flag_rgbd_3D_3_clicked();

    void serial_terminal_ReadData();


    void on_ARM2Enable_2_clicked(bool checked);

    void on_ARM1Enable_2_clicked(bool checked);


    void on_Arm_L_X_posi_pressed();

    void on_Arm_L_X_nega_pressed();

    void on_Arm_L_Y_posi_pressed();

    void on_Arm_L_Y_nega_pressed();

    void on_Arm_L_Z_posi_pressed();

    void on_Arm_L_Z_nega_pressed();

    void on_Arm_L_X_posi_released();

    void on_Arm_L_X_nega_released();

    void on_Arm_L_Y_posi_released();

    void on_Arm_L_Y_nega_released();

    void on_Arm_L_Z_posi_released();

    void on_Arm_L_Z_nega_released();

    void on_Arm_Stop_All_4_clicked(bool check);

    void on_Arm_R_X_posi_pressed();

    void on_Arm_R_X_nega_pressed();

    void on_Arm_R_Y_posi_pressed();

    void on_Arm_R_Y_nega_pressed();

    void on_Arm_R_Z_posi_pressed();

    void on_Arm_R_Z_nega_pressed();

    void on_Arm_R_X_posi_released();

    void on_Arm_R_X_nega_released();

    void on_Arm_R_Y_posi_released();

    void on_Arm_R_Y_nega_released();

    void on_Arm_R_Z_posi_released();

    void on_Arm_R_Z_nega_released();

    void on_Jog_Step_2_sliderReleased();

    void on_Arm_Stop(bool check);

    void on_Arm_L_Joint_Stop_clicked(bool check);

    void on_Arm_Stop_All_5_clicked(bool check);

    void on_Arm_L_1_posi_pressed();

    void on_Arm_L_1_nega_pressed();

    void on_Arm_L_2_posi_pressed();

    void on_Arm_L_2_nega_pressed();

    void on_Arm_L_3_posi_pressed();

    void on_Arm_L_3_nega_pressed();

    void on_Arm_L_4_posi_pressed();

    void on_Arm_L_4_nega_pressed();

    void on_Arm_L_5_posi_pressed();

    void on_Arm_L_5_nega_pressed();

    void on_Arm_L_6_posi_pressed();

    void on_Arm_L_6_nega_pressed();

    void Arm_L_Joint_released();

    void on_Arm_L_1_posi_released();

    void on_Arm_L_1_nega_released();

    void on_Arm_L_2_posi_released();

    void on_Arm_L_2_nega_released();

    void on_Arm_L_3_posi_released();

    void on_Arm_L_3_nega_released();

    void on_Arm_L_4_posi_released();

    void on_Arm_L_4_nega_released();

    void on_Arm_L_5_posi_released();

    void on_Arm_L_5_nega_released();

    void on_Arm_L_6_posi_released();

    void on_Arm_L_6_nega_released();

    void on_ARM2Enable_3_clicked(bool checked);

    void on_Arm_R_Joint_Stop_clicked(bool check);

    void on_Arm_Stop_All_6_clicked(bool check);

    void on_Arm_R_1_posi_pressed();

    void on_Arm_R_1_nega_pressed();

    void on_Arm_R_2_posi_pressed();

    void on_Arm_R_2_nega_pressed();

    void on_Arm_R_3_posi_pressed();

    void on_Arm_R_3_nega_pressed();

    void on_Arm_R_4_posi_pressed();

    void on_Arm_R_4_nega_pressed();

    void on_Arm_R_5_posi_pressed();

    void on_Arm_R_5_nega_pressed();

    void on_Arm_R_6_posi_pressed();

    void on_Arm_R_6_nega_pressed();

    void Arm_R_Joint_released();

    void on_Arm_R_1_posi_released();

    void on_Arm_R_1_nega_released();

    void on_Arm_R_2_posi_released();

    void on_Arm_R_2_nega_released();

    void on_Arm_R_3_posi_released();

    void on_Arm_R_3_nega_released();

    void on_Arm_R_4_posi_released();

    void on_Arm_R_4_nega_released();

    void on_Arm_R_5_posi_released();

    void on_Arm_R_5_nega_released();

    void on_Arm_R_6_posi_released();

    void on_Arm_R_6_nega_released();

    void on_ARM1Enable_3_clicked(bool checked);

    void on_Jog_Step_3_sliderReleased();

    void on_Jog_Step_4_sliderReleased();

    void on_ARM_Reset_clicked();

    void on_ARM_Point_Continue_clicked();

    void on_ARM_Point_Continue_2_clicked();


    void on_Terminal_Face_Update_clicked();

    void on_Car_Speed_Slide_sliderReleased();

    void on_Shotdown_clicked();


    void on_ARM_Point_Continue_3_clicked();

    void on_ARM_Point_Continue_4_clicked();

    void on_gcamera_tabBarClicked(int index);

    void on_scan_port_button_terminal_clicked();

    bool scan_port_list_terminal();

    void on_serial_connect_button_terminal_clicked();

    void on_Both_Arm_Position_1_clicked();

    void on_Both_Arm_Position_2_clicked();

    void on_Both_Arm_Position_3_clicked();

    void on_Both_Arm_Position_4_clicked();

    void on_Right_Arm_Position_4_clicked();

    void on_Right_Arm_Position_5_clicked();

    void on_Arm_Tool_0_toggled(bool checked);

    void on_gripper_force_sliderReleased();

    void on_gripper_position_sliderReleased();

    void on_tabWidget_2_tabBarClicked(int index);

    void on_ExpShow_clicked(bool checked);

    void on_flywheel_Out_clicked();

    void on_flywheel_In_clicked();

    void Enable_Tool_All();

    void Disable_Tool_All();

    void pingfail_handle();


private:
    Ui::MainWindow *ui;
    Dialog *dialog;
    bool is_dialog_launch;
    ImageProcessor image_processor;
    Show3D show3D;
    Show3D show3D_1;
    Show3D show3D_2;
    ImageShower* image_shower;

    /*Global Camera*/
    GlobalCamera *gcamera;
    cv::Mat img_gcamera;


    /*Communication*/
//    MsgInfo *msg_info;
    SocketRobot *socket_robot;

    /*RealtimeCommunication*/
//    RealtimeCommunication * realtime_communication;

    /*Serial*/
    Serial *serial;
    bool serial_is_ok;   
    Serial *serial_terminal;
    Terminal terminal;

    /*robot*/
    Arm_pose arm1_last_pose;
    Arm_pose arm1_curr_pose;
    Arm_pose arm2_last_pose;
    Arm_pose arm2_curr_pose;

    cv::Mat img_d435_rgb;
    cv::Mat img_d435_depth;
    cv::Mat img_d435_color_depth;

    cv::Mat img_mono_left;
    cv::Mat img_mono_right;

    cv::Mat img_mono_omni;

    cv::Mat img_d435_rgb1;
    cv::Mat img_d435_depth1;
    cv::Mat img_d435_color_depth1;

    cv::Mat img_d435_rgb2;
    cv::Mat img_d435_depth2;
    cv::Mat img_d435_color_depth2;

    /*watch dog*/
    UIWatchDog *watchdog;

    /*ping check*/
    PingCheck *pingcheck;
};

#endif // MAINWINDOW_H
