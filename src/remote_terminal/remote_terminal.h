#ifndef TERMINAL_COMMON_H
#define TERMINAL_COMMON_H

#include <QDebug>
#include <map>
#include <unordered_map>
#include <iostream>
#include <math.h>
#include <iostream>
#include <sstream>

#define     PRESS               '1'
#define     RELEASE             '0'

#define     VEHICLE_RADIUS      10.0
#define     VEHICLE_X_MIN       6.0
#define     VEHICLE_X_MAX       255.0
#define     VEHICLE_Y_MIN       6.0
#define     VEHICLE_Y_MAX       255.0
#define     VEHICLE_Z_MIN       6.0
#define     VEHICLE_Z_MAX       255.0

#define     ROLL_Limit          0.5

#define     pi                  3.1415926
#define     sqrt_2              0.7071

#define     SENT_ARM_BOOL_CNT_MAX 2

using namespace std;


class Terminal
{
public:
    explicit Terminal();
    virtual ~Terminal();
    void init_frame_table();
    bool receive_data(QString buf);
    bool data_save();
    void data_split();
    bool no_near_zero(QString str1, QString str2);
    bool check_frame_start_end();
    bool check_sum();
    void show_data();
    int QString2HexInt_single(QString str);
    QByteArray QString2HexQString(QString str);
    char* QString2BinChar(QString str);
    QString getXORresult(QString str1, QString str2);
    bool if_vehicle_stick_change();
    /*-------------------------------*/
    bool if_roll2_change();
    bool if_roll1_change();

    // 自锁按钮控制
    int self_lock_button_control(bool statue, char order_old, char order_new);
    int get_vehicle_enable_order();
    // byte 命令存储
    void cmd1_save();
    void cmd2_save();
    void cmd3_save();
    void cmd4_save();
    void cmd5_save();
    void cmd6_save();
    void cmd7_save();
    void cmd8910_save();
    void cmd111213_save();
    void cmd141516_save();

    string get_arm_joint_space_status();

    // bit 命令存储
    QString data;
    //cmd_1
    char cmd_point_or_continue; char cmd_point_or_continue_old;
        //BIT1 do not understand
    char change_tool_enable;    char change_tool_enable_old;//cmd_ptz_focus_more
    char cmd_ptz_focus_less;    char cmd_ptz_focus_less_old;
    char cmd_ptz_zoom_more;     char cmd_ptz_zoom_more_old;
    char cmd_ptz_zoom_less;     char cmd_ptz_zoom_less_old;
    char cmd_launch_camera;     char cmd_launch_camera_old;
    char cmd_light;             char cmd_light_old;
    //cmd_2
    char cmd_vehicle_enable;    char cmd_vehicle_enable_old;
    bool vehicle_statue;
    char cmd_left_arm_enable;   char cmd_left_arm_enable_old;
    char cmd_right_arm_enable;  char cmd_right_arm_enable_old;
    char cmd_hand_open;         char cmd_hand_open_old;
    char cmd_hand_close;        char cmd_hand_close_old;
    char cmd_arms_init_pose;    char cmd_arms_init_pose_old;
    char cmd_arms_end_pose;     char cmd_arms_end_pose_old;
        //BIT7 undefine
    //cmd_3
    char cmd_onekey_can;        char cmd_onekey_can_old;
    char cmd_onekey_balance;    char cmd_onekey_balance_old;
    char cmd_onekey_disposal;   char cmd_onekey_disposal_old;
    char cmd_yuntai_left_move;  char cmd_yuntai_left_move_old;
    char cmd_yuntai_right_move; char cmd_yuntai_right_move_old;
    char cmd_arm1_arm2_change;  char cmd_arm1_arm2_change_old;
    char cmd_yuntai_up_move;    char cmd_yuntai_up_move_old;
        //BIT7 undefine
    //cmd_4
    char cmd_robot_control_stop;    char cmd_robot_control_stop_old;
    char cmd_left_arm_stop;         char cmd_left_arm_stop_old;
    char cmd_right_arm_stop;        char cmd_right_arm_stop_old;
    char cmd_all_stop;              char cmd_all_stop_old;
    char cmd_arm_forward_1_true;    char cmd_arm_forward_1_true_old;
    char cmd_arm_left_2_true;       char cmd_arm_left_2_true_old;
    char cmd_arm_up_3_true;         char cmd_arm_up_3_true_old;
    char cmd_arm_x_true_4_true;     char cmd_arm_x_true_4_true_old;
    //cmd_5
    char cmd_arm_y_true_5_true;     char cmd_arm_y_true_5_true_old;
    char cmd_arm_z_true_6_true;     char cmd_arm_z_true_6_true_old;
    char cmd_arm_back_1_false;      char cmd_arm_back_1_false_old;
    char cmd_arm_right_2_false;     char cmd_arm_right_2_false_old;
    char cmd_arm_down_3_false;      char cmd_arm_down_3_false_old;
    char cmd_arm_x_false_4_false;   char cmd_arm_x_false_4_false_old;
    char cmd_arm_y_false_5_false;   char cmd_arm_y_false_5_false_old;
    char cmd_arm_z_false_6_false;   char cmd_arm_z_false_6_false_old;
    //cmd_6
        //BIT0 do not understand
    char cmd_arm_tool_4;            char cmd_arm_tool_4_old;    //char cmd_climb; char cmd_climb_old;
    char cmd_arm_tool_3;            char cmd_arm_tool_3_old;    //char cmd_surmount; char cmd_surmount_old;
    char cmd_arm_tool_2;            char cmd_arm_tool_2_old;    //char cmd_stair; char cmd_stair_old;
    char cmd_arm_tool_1;            char cmd_arm_tool_1_old;    //char cmd_uneven; char cmd_uneven_old;
    char cmd_Arm_Reset;             char cmd_Arm_Reset_old;
    char cmd_yuntai_down_move;      char cmd_yuntai_down_move_old;
    char cmd_joint_space_change;        char cmd_joint_space_change_old; // 0:task space 1:joint space
    //cmd_7
    char cmd_roll1_button;     char cmd_roll1_button_old;
        //BIT1 do not understand
    char cmd_roll2_button;     char cmd_roll2_button_old;
        //BIT3 undefined
        //BIT4 undefined
        //BIT5 undefined
        //BIT6 undefined
        //BIT7 undefined
    //cmd_8910
    double cmd_roll2_x, cmd_roll2_x_old;
    double cmd_roll2_y, cmd_roll2_y_old;
    double cmd_roll2_z, cmd_roll2_z_old;
    //cmd_111213
    double cmd_vehicle_theta, cmd_vehicle_theta_old; //xy
    double cmd_vehicle_pho, cmd_vehicle_pho_old; //xy
    double cmd_vehicle_self_rot, cmd_vehicle_self_rot_old; //z
    //cmd_141516
    double cmd_roll1_x, cmd_roll1_x_old;
    double cmd_roll1_y, cmd_roll1_y_old;
    double cmd_roll1_z, cmd_roll1_z_old;

    // flag
    bool flag_cmd_1_change;
    bool flag_cmd_2_change;
    bool flag_cmd_3_change;
    bool flag_cmd_4_change;
    bool flag_cmd_5_change;
    bool flag_cmd_6_change;
    bool flag_cmd_7_change;
    bool flag_cmd_8910_change;
    bool flag_cmd_111213_change;
    bool flag_cmd_141516_change;
    bool flag_right_vert;
    bool flag_left_vert;
    bool flag_arm_bool_move_stop;
    int sent_arm_bool_cnt;
    int tool_num;


private:
    QString FEAME_FIRST;
    QString FEAME_END_1;
    QString FEAME_END_2;

    bool flag_x_zero;
    bool flag_y_zero;
    bool flag_z_zero;

    int VEHICLE_X_ZERO;
    int VEHICLE_Y_ZERO;
    int VEHICLE_Z_ZERO;

    /*------------------------------*/
    bool flag_x_roll2_zero;
    bool flag_y_roll2_zero;
    bool flag_z_roll2_zero;

    int roll2_X_ZERO;
    int roll2_Y_ZERO;
    int roll2_Z_ZERO;



    /*------------------------------*/

    bool flag_x_roll1_zero;
    bool flag_y_roll1_zero;
    bool flag_z_roll1_zero;

    int roll1_X_ZERO;
    int roll1_Y_ZERO;
    int roll1_Z_ZERO;



    QString frame_start;
    QString frame_cmd[16];
    QString frame_cmd_old[16];
    QString frame_checksum;
    QString frame_end_1;
    QString frame_end_2;

};


#endif
