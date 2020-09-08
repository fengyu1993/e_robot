#include "remote_terminal.h"

using namespace std;

Terminal::Terminal()
{
    FEAME_FIRST = "7e";
    FEAME_END_1 = "0d";
    FEAME_END_2 = "0a";

    flag_x_zero = false;
    flag_y_zero = false;
    flag_z_zero = false;

    flag_cmd_1_change = false;
    flag_cmd_2_change = false;
    flag_cmd_3_change = false;
    flag_cmd_4_change = false;
    flag_cmd_5_change = false;
    flag_cmd_6_change = false;
    flag_cmd_7_change = false;
    flag_cmd_8910_change = false;
    flag_cmd_111213_change = false;
    flag_cmd_141516_change = false;

    flag_right_vert = false;
    flag_left_vert = false;

    flag_arm_bool_move_stop = false;
    sent_arm_bool_cnt = 0;
    tool_num = 0;

    //cmd_1
    cmd_point_or_continue = RELEASE;
    change_tool_enable = RELEASE;
    cmd_ptz_focus_less = RELEASE;
    cmd_ptz_zoom_more = RELEASE;
    cmd_ptz_zoom_less = RELEASE;
    cmd_launch_camera = RELEASE;
    cmd_light = RELEASE;
    //cmd_2
    cmd_vehicle_enable = RELEASE;
    vehicle_statue = false;
    cmd_left_arm_enable = RELEASE;
    cmd_right_arm_enable = RELEASE;
    cmd_hand_open = RELEASE;
    cmd_hand_close = RELEASE;
    cmd_arms_init_pose = RELEASE;
    cmd_arms_end_pose = RELEASE;
    //cmd_3
    cmd_onekey_can = RELEASE;
    cmd_onekey_balance = RELEASE;
    cmd_onekey_disposal = RELEASE;
    cmd_yuntai_left_move = RELEASE;
    cmd_yuntai_right_move = RELEASE;
    cmd_arm1_arm2_change = RELEASE;
    cmd_yuntai_up_move = RELEASE;
    //cmd_4
    cmd_robot_control_stop = RELEASE;
    cmd_left_arm_stop = RELEASE;
    cmd_right_arm_stop = RELEASE;
    cmd_all_stop = RELEASE;
    cmd_arm_forward_1_true = RELEASE;
    cmd_arm_left_2_true = RELEASE;
    cmd_arm_up_3_true = RELEASE;
    cmd_arm_x_true_4_true = RELEASE;
    //cmd_5    
    cmd_arm_y_true_5_true = RELEASE;
    cmd_arm_z_true_6_true = RELEASE;
    cmd_arm_back_1_false = RELEASE;
    cmd_arm_right_2_false = RELEASE;
    cmd_arm_down_3_false = RELEASE;
    cmd_arm_x_false_4_false = RELEASE;
    cmd_arm_y_false_5_false = RELEASE;
    cmd_arm_z_false_6_false = RELEASE;
    //cmd_6
    cmd_arm_tool_4 = RELEASE;    //cmd_climb = RELEASE;
    cmd_arm_tool_3 = RELEASE;    //cmd_surmount = RELEASE;
    cmd_arm_tool_2 = RELEASE;    //cmd_stair = RELEASE;
    cmd_arm_tool_1 = PRESS;    //cmd_uneven = RELEASE;
    cmd_Arm_Reset = RELEASE;
    cmd_yuntai_down_move = RELEASE;
    cmd_joint_space_change = RELEASE;   // task space
    //cmd_7
    cmd_roll1_button = RELEASE;
    cmd_roll2_button = RELEASE;
    //cmd_8910
    cmd_roll2_x = 0;        cmd_roll2_x_old = 0;
    cmd_roll2_y = 0;        cmd_roll2_y_old = 0;
    cmd_roll2_z = 0;        cmd_roll2_z_old = 0;
    //cmd_111213
    cmd_vehicle_theta  = 0;     cmd_vehicle_theta_old = 0;  //xy
    cmd_vehicle_pho = 0;        cmd_vehicle_pho_old = 0;    //xy
    cmd_vehicle_self_rot = 0;   cmd_vehicle_self_rot_old = 0;   //z

    //cmd_141516
    cmd_roll1_x = 0;        cmd_roll1_x_old = 0;
    cmd_roll1_y = 0;        cmd_roll1_y_old = 0;
    cmd_roll1_z = 0;        cmd_roll1_z_old = 0;

    //
}

Terminal::~Terminal()
{

}

bool Terminal::receive_data(QString buf)
{
    bool flag = true;
    return flag;
}

void Terminal::show_data()
{
    cout << "RS232 Recieve:" << endl;
    cout << data.toLatin1().data() << endl;

        cout << "frame_start; " << frame_start.toLatin1().data() << endl;
        cout << "data: ";
        for (int i=0; i<16; i++)
        {
            cout << frame_cmd[i].toLatin1().data();
        }
        cout << endl;
        cout << "frame_checksum: " << frame_checksum.toLatin1().data() << endl;
        cout << "frame_end_1: " << frame_end_1.toLatin1().data() << endl;
        cout << "frame_end_2: " << frame_end_2.toLatin1().data() << endl;
        cout << "check_flag: " << check_sum() << endl;
        cout << "check: " << check_sum() <<endl;
}

bool Terminal::data_save()
{
    bool flag_suc = false;
    data_split();


//    show_data();

    if( check_sum() )
    {
        if(frame_cmd[0] != frame_cmd_old[0])
        {
            cmd1_save();
            flag_cmd_1_change = true;
        }
        if(frame_cmd[1] != frame_cmd_old[1])
        {
            cmd2_save();
            flag_cmd_2_change = true;
        }
        if(frame_cmd[2] != frame_cmd_old[2])
        {
            cmd3_save();
            flag_cmd_3_change = true;
        }
        if(frame_cmd[3] != frame_cmd_old[3])
        {
            cmd4_save();
            flag_cmd_4_change = true;
        }
        if(frame_cmd[4] != frame_cmd_old[4])
        {
            cmd5_save();
            flag_cmd_5_change = true;
        }
        if(frame_cmd[5] != frame_cmd_old[5])
        {
            cmd6_save();
            flag_cmd_6_change = true;
        }
        if(frame_cmd[6] != frame_cmd_old[6])
        {
            cmd7_save();
            flag_cmd_7_change = true;
        }
        if(no_near_zero(frame_cmd[7], frame_cmd_old[7])
           || no_near_zero(frame_cmd[8], frame_cmd_old[8])
           || no_near_zero(frame_cmd[9], frame_cmd_old[9]))
        {
            cmd8910_save();
            flag_cmd_8910_change = true;
        }
        if(no_near_zero(frame_cmd[10], frame_cmd_old[10])
           || no_near_zero(frame_cmd[11], frame_cmd_old[11])
           || no_near_zero(frame_cmd[12], frame_cmd_old[12]))
        {
            cmd111213_save();
            flag_cmd_111213_change = true;
        }
        if(no_near_zero(frame_cmd[13], frame_cmd_old[13])
                || no_near_zero(frame_cmd[14], frame_cmd_old[14])
                || no_near_zero(frame_cmd[15], frame_cmd_old[15]) )
        {
            cmd141516_save();
            flag_cmd_141516_change = true;
        }
    }

    flag_suc = flag_cmd_1_change | flag_cmd_2_change | flag_cmd_3_change
            | flag_cmd_4_change | flag_cmd_5_change | flag_cmd_6_change
            | flag_cmd_7_change | flag_cmd_8910_change
            | flag_cmd_111213_change | flag_cmd_141516_change;
    flag_suc = flag_suc | flag_arm_bool_move_stop;
    return flag_suc;
}

bool Terminal::no_near_zero(QString str1, QString str2)
{
    int a = QString2HexInt_single(str1);
    int b = QString2HexInt_single(str2);

    if (abs(a-b) > 2)
        return true;
    else
        return false;

}

void Terminal::data_split()
{
        frame_start = data.mid(0, 2);

        for(int i=0; i<16; i++)
        {
            frame_cmd_old[i] = frame_cmd[i];
            frame_cmd[i] = data.mid(2*(i+1), 2);
        }

        frame_checksum = data.mid(34, 2);
        frame_end_1 = data.mid(36, 2);
        frame_end_2 = data.mid(38, 2);
}

bool Terminal::check_frame_start_end()
{
    QByteArray bs = data.toLatin1();
    bool flag_finish = false;
    int size = data.size();
    if(size == 40)
    {
        if(bs[0]=='7' && bs[1]=='e'
                && bs[36]=='0' && bs[37]=='d'
                && bs[38]=='0' && bs[39]=='a')
        {
            flag_finish = true;
        }
    }
    else if(size > 40)
    {
        //////////////////////////////////////

           int i = bs.lastIndexOf("0d0a");
           bs = bs.mid(i-36,40);
           if (bs.startsWith("7e"))
           {
               data = bs;
               flag_finish = true;
           }
           else
           {
               data.clear();
           }


        //////////////////////////////////////

    }

    return flag_finish;
}

bool Terminal::check_sum()
{
    QString check = frame_start;
    for(int i=0; i<16; i++)
    {
        check = getXORresult(check, frame_cmd[i]);
    }

    if(check.compare(frame_checksum) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int Terminal::QString2HexInt_single(QString str)
{
    QByteArray PublishPlotocal, tempByteHight, tempByteLow;
    PublishPlotocal = str.toLatin1();

    tempByteHight[0] = PublishPlotocal.at(0);
    int numHigh = tempByteHight.toInt(NULL, 16);

    tempByteLow[0] = PublishPlotocal.at(1);
    int numLow = tempByteLow.toInt(NULL, 16);

    int Byte = numHigh*16 + numLow;
    return Byte;
}

QByteArray Terminal::QString2HexQString(QString str)
{
    QByteArray buf;
    int array[20];
    for (int i=0; i<20; i++)
    {
        array[i] = QString2HexInt_single(str.mid(2*i, 2));
        buf.append(QString::number(array[i]));
    }
    return buf;
}

QString Terminal::getXORresult(QString str1, QString str2)
{
    int a = QString2HexInt_single(str1);
    int b = QString2HexInt_single(str2);
    int c = a ^ b;

    char buffer[20];
    sprintf(buffer, "%02x",c);
    string out(buffer);

    QString result = QString::fromStdString(out);

    return result;
}

bool Terminal::if_vehicle_stick_change()
{
    bool flag1 = cmd_vehicle_theta == cmd_vehicle_theta_old; //xy
    bool flag2 = cmd_vehicle_pho == cmd_vehicle_pho_old; //xy
    bool flag3 =  cmd_vehicle_self_rot == cmd_vehicle_self_rot_old; //z
    if(flag1 && flag2 && flag3)
        return false;
    else
        return true;
}
/*----------------------------------------*/
bool Terminal::if_roll2_change()
{
    bool flag1 = cmd_roll2_x == cmd_roll2_x_old; //xy
    bool flag2 = cmd_roll2_y== cmd_roll2_y_old; //xy
    bool flag3 =  cmd_roll2_z== cmd_roll2_z_old; //z



    if(flag1 && flag2 && flag3)
        return false;
    else
        return true;
}

bool Terminal::if_roll1_change()
{
    bool flag1 = cmd_roll1_x == cmd_roll1_x_old; //xy
    bool flag2 = cmd_roll1_y== cmd_roll1_y_old; //xy
    bool flag3 =  cmd_roll1_z== cmd_roll1_z_old; //z
    if(flag1 && flag2 && flag3)
        return false;
    else
        return true;
}


int Terminal::self_lock_button_control(bool statue, char order_old, char order_new)
{
    int flag;
    bool flag_new, flag_old;

    if(order_new == PRESS)
        flag_new = true;
    else if(order_new == RELEASE)
        flag_new = false;
    else
        return 2;

    if(order_old == PRESS)
        flag_old = true;
    else if(order_old == RELEASE)
        flag_old = false;
    else
        return 2;

    flag = ((!statue) & flag_old & (!flag_new))
            | (statue & (!flag_old)) | (statue & flag_new);

    return flag;
}

int Terminal::get_vehicle_enable_order()
{
    return  self_lock_button_control
                (vehicle_statue, cmd_vehicle_enable_old, cmd_vehicle_enable);
}
char* Terminal::QString2BinChar(QString str)
{
    int temp = QString2HexInt_single(str);
    QString str2= QString::number(temp);
    // 前位补0
    QString strNew2 = QString("%1").arg(str2.toInt(), 8, 2, QLatin1Char('0'));
    QByteArray ba = strNew2.toLatin1();
    char* byte = ba.data();

    return byte;
}

void Terminal::cmd1_save()
{
        char* byte = QString2BinChar(frame_cmd[0]);

        cmd_point_or_continue_old = cmd_point_or_continue;
        cmd_point_or_continue = byte[7];

        change_tool_enable_old = change_tool_enable;
        change_tool_enable = byte[5];


        cmd_ptz_focus_less_old = cmd_ptz_focus_less;
        cmd_ptz_focus_less = byte[4];

        cmd_ptz_zoom_more_old = cmd_ptz_zoom_more;
        cmd_ptz_zoom_more = byte[3];

        cmd_ptz_zoom_less_old = cmd_ptz_zoom_less;
        cmd_ptz_zoom_less = byte[2];

        cmd_launch_camera_old = cmd_launch_camera;
        cmd_launch_camera = byte[1];

        cmd_light_old = cmd_light;
        cmd_light = byte[0];

//        cout << "cmd_1: " << byte << endl;
//        cout << "change_tool_enable: " << change_tool_enable << endl;
//        cout << "cmd_ptz_focus_less: " << cmd_ptz_focus_less << endl;
//        cout << "cmd_ptz_zoom_more: " << cmd_ptz_zoom_more << endl;
//        cout << "cmd_ptz_zoom_less: " << cmd_ptz_zoom_less << endl;
//        cout << "cmd_launch_camera: " << cmd_launch_camera << endl;
//        cout << "cmd_light: " << cmd_light << endl;
}

void Terminal::cmd2_save()
{
    char* byte = QString2BinChar(frame_cmd[1]);

    cmd_vehicle_enable_old = cmd_vehicle_enable;
    cmd_vehicle_enable = byte[7];

    cmd_left_arm_enable_old = cmd_left_arm_enable;
    cmd_left_arm_enable = byte[6];

    cmd_right_arm_enable_old = cmd_right_arm_enable;
    cmd_right_arm_enable = byte[5];

    cmd_hand_open_old = cmd_hand_open;
    cmd_hand_open = byte[4];

    cmd_hand_close_old = cmd_hand_close;
    cmd_hand_close = byte[3];

    cmd_arms_init_pose_old = cmd_arms_init_pose;
    cmd_arms_init_pose = byte[2];

    cmd_arms_end_pose_old = cmd_arms_end_pose;
    cmd_arms_end_pose = byte[1];

//    cout << "cmd_2: " << byte << endl;
//    cout << "cmd_vehicle_enable: " << cmd_vehicle_enable << endl;
//    cout << "cmd_left_arm_enable: " << cmd_left_arm_enable << endl;
//    cout << "cmd_right_arm_enable: " << cmd_right_arm_enable << endl;
//    cout << "cmd_hand_open: " << cmd_hand_open << endl;
//    cout << "cmd_hand_close: " << cmd_hand_close << endl;
//    cout << "cmd_arms_init_pose_old: " << cmd_arms_init_pose_old << endl;
//    cout << "cmd_arms_init_pose: " << cmd_arms_init_pose << endl;
//    cout << "cmd_arms_end_pose_old: " << cmd_arms_end_pose_old << endl;
//    cout << "cmd_arms_end_pose: " << cmd_arms_end_pose << endl;

}

void Terminal::cmd3_save()
{
    char* byte = QString2BinChar(frame_cmd[2]);

    cmd_onekey_can_old = cmd_onekey_can;
    cmd_onekey_can = byte[7];

    cmd_onekey_balance_old = cmd_onekey_balance;
    cmd_onekey_balance = byte[6];

    cmd_onekey_disposal_old = cmd_onekey_disposal;
    cmd_onekey_disposal = byte[5];

    cmd_yuntai_left_move_old = cmd_yuntai_left_move;
    cmd_yuntai_left_move = byte[4];

    cmd_yuntai_right_move_old = cmd_yuntai_right_move;
    cmd_yuntai_right_move = byte[3];

    cmd_arm1_arm2_change_old = cmd_arm1_arm2_change;
    cmd_arm1_arm2_change = byte[2];

    cmd_yuntai_up_move_old = cmd_yuntai_up_move;
    cmd_yuntai_up_move = byte[1];

//    cout << "cmd_3: " << byte << endl;
//    cout << "cmd_onekey_can: " << cmd_onekey_can << endl;
//    cout << "cmd_onekey_balance: " << cmd_onekey_balance << endl;
//    cout << "cmd_onekey_disposal: " << cmd_onekey_disposal << endl;
//    cout << "cmd_arm1_arm2_change: " << cmd_arm1_arm2_change << endl;
}

void Terminal::cmd4_save()
{
    char* byte = QString2BinChar(frame_cmd[3]);

    cmd_robot_control_stop_old = cmd_robot_control_stop;
    cmd_robot_control_stop = byte[7];

    cmd_left_arm_stop_old = cmd_left_arm_stop;
    cmd_left_arm_stop = byte[6];

    cmd_right_arm_stop_old = cmd_right_arm_stop;
    cmd_right_arm_stop = byte[5];

    cmd_all_stop_old = cmd_all_stop;
    cmd_all_stop = byte[4];

    cmd_arm_forward_1_true_old = cmd_arm_forward_1_true;
    cmd_arm_forward_1_true = byte[3];

    cmd_arm_left_2_true_old = cmd_arm_left_2_true;
    cmd_arm_left_2_true = byte[2];

    cmd_arm_up_3_true_old = cmd_arm_up_3_true;
    cmd_arm_up_3_true = byte[1];

    cmd_arm_x_true_4_true_old = cmd_arm_x_true_4_true;
    cmd_arm_x_true_4_true = byte[0];


//    cout << "cmd_4: " << byte << endl;
//    cout << "cmd_robot_control_stop: " << cmd_robot_control_stop << endl;
//    cout << "cmd_left_arm_stop: " << cmd_left_arm_stop << endl;
//    cout << "cmd_right_arm_stop: " << cmd_right_arm_stop << endl;
//    cout << "cmd_all_stop: " << cmd_all_stop << endl;
//    cout << "cmd_left_arm_forward: " << cmd_left_arm_forward << endl;
//    cout << "cmd_left_arm_back: " << cmd_left_arm_back << endl;
//    cout << "cmd_left_arm_left: " << cmd_left_arm_left << endl;
//    cout << "cmd_left_arm_right: " << cmd_left_arm_right << endl;

}

void Terminal::cmd5_save()
{
    char* byte = QString2BinChar(frame_cmd[4]);

    cmd_arm_y_true_5_true_old = cmd_arm_y_true_5_true;
    cmd_arm_y_true_5_true = byte[7];

    cmd_arm_z_true_6_true_old = cmd_arm_z_true_6_true;
    cmd_arm_z_true_6_true = byte[6];

    cmd_arm_back_1_false_old = cmd_arm_back_1_false;
    cmd_arm_back_1_false = byte[5];

    cmd_arm_right_2_false_old = cmd_arm_right_2_false;
    cmd_arm_right_2_false = byte[4];

    cmd_arm_down_3_false_old = cmd_arm_down_3_false;
    cmd_arm_down_3_false = byte[3];

    cmd_arm_x_false_4_false_old = cmd_arm_x_false_4_false;
    cmd_arm_x_false_4_false = byte[2];

    cmd_arm_y_false_5_false_old = cmd_arm_y_false_5_false;
    cmd_arm_y_false_5_false = byte[1];

    cmd_arm_z_false_6_false_old = cmd_arm_z_false_6_false;
    cmd_arm_z_false_6_false = byte[0];

//        cout << "cmd_5: " << byte << endl;
//        cout << "cmd_arm_y_true_5_true: " << cmd_arm_y_true_5_true << endl;
//        cout << "cmd_arm_z_true_6_true: " << cmd_arm_z_true_6_true << endl;
//        cout << "cmd_arm_back_1_false: " << cmd_arm_back_1_false << endl;
//        cout << "cmd_arm_right_2_false: " << cmd_arm_right_2_false << endl;
//        cout << "cmd_arm_down_3_false: " << cmd_arm_down_3_false << endl;
//        cout << "cmd_arm_x_false_4_false: " << cmd_arm_x_false_4_false << endl;
//        cout << "cmd_arm_y_false_5_false: " << cmd_arm_y_false_5_false << endl;
//        cout << "cmd_arm_z_false_6_false: " << cmd_arm_z_false_6_false << endl;

}

void Terminal::cmd6_save()
{
    char* byte = QString2BinChar(frame_cmd[5]);

    cmd_arm_tool_4_old = cmd_arm_tool_4; //cmd_climb_old = cmd_climb;
    cmd_arm_tool_4 = byte[6];             //cmd_climb = byte[6];

    cmd_arm_tool_3_old = cmd_arm_tool_3; //cmd_surmount_old = cmd_surmount;
    cmd_arm_tool_3 = byte[5];             //cmd_surmount = byte[5];

    cmd_arm_tool_2_old = cmd_arm_tool_2; //cmd_stair_old = cmd_stair;
    cmd_arm_tool_2 = byte[4];             //cmd_stair = byte[4];

    cmd_arm_tool_1_old = cmd_arm_tool_1; //cmd_uneven_old = cmd_uneven;
    cmd_arm_tool_1 = byte[3];             //cmd_uneven = byte[3];

    cmd_Arm_Reset_old = cmd_Arm_Reset;
    cmd_Arm_Reset = byte[2];

    cmd_yuntai_down_move_old = cmd_yuntai_down_move;
    cmd_yuntai_down_move = byte[1];

    cmd_joint_space_change_old = cmd_joint_space_change;
    cmd_joint_space_change = byte[0];

//    cout << "cmd_6: " << byte << endl;
//    cout << "cmd_climb: " << cmd_climb << endl;
//    cout << "cmd_surmount: " << cmd_surmount << endl;
//    cout << "cmd_stair: " << cmd_stair << endl;
//    cout << "cmd_uneven: " << cmd_uneven << endl;
//    cout << "cmd_joint_space_change: " << cmd_joint_space_change << endl;

}

void Terminal::cmd7_save()
{
    char *byte = QString2BinChar(frame_cmd[6]);
    cmd_roll1_button_old = cmd_roll1_button;
    cmd_roll1_button = byte[7];
    cmd_roll2_button_old = cmd_roll2_button;
    cmd_roll2_button = byte[5];
}

void Terminal::cmd8910_save()
{

    cmd_roll2_x_old = cmd_roll2_x;
    cmd_roll2_y_old = cmd_roll2_y;
    cmd_roll2_z_old = cmd_roll2_z;

    int temp_y = QString2HexInt_single(frame_cmd[7]);
    int temp_x = QString2HexInt_single(frame_cmd[8]);
    int temp_z = QString2HexInt_single(frame_cmd[9]);

    if (flag_x_roll2_zero == false)
    {
        flag_x_roll2_zero = true;
        roll2_X_ZERO = temp_x;
    }
    if (flag_y_roll2_zero == false)
    {
        flag_y_roll2_zero = true;
        roll2_Y_ZERO = temp_y;
    }
    if (flag_z_roll2_zero == false)
    {
        flag_z_roll2_zero = true;
        roll2_Z_ZERO = temp_z;
    }


    if (temp_x > roll2_X_ZERO)
        cmd_roll2_x = 1 / (VEHICLE_X_MAX - roll2_X_ZERO) * (temp_x - roll2_X_ZERO);
    else
        cmd_roll2_x = 1 / (roll2_X_ZERO - VEHICLE_X_MIN) * (temp_x - roll2_X_ZERO);


    if (temp_y > roll2_Y_ZERO)
        cmd_roll2_y = 1 / (VEHICLE_Y_MAX - roll2_Y_ZERO) * (temp_y - roll2_Y_ZERO);
    else
        cmd_roll2_y = 1 / (roll2_Y_ZERO - VEHICLE_Y_MIN) * (temp_y - roll2_Y_ZERO);


    if (temp_z > roll2_Z_ZERO)
        cmd_roll2_z = 1 / (VEHICLE_Z_MAX - roll2_Z_ZERO) * (temp_z - roll2_Z_ZERO);
    else
        cmd_roll2_z = 1 / (roll2_Z_ZERO - VEHICLE_Z_MIN) * (temp_z - roll2_Z_ZERO);

//    cout << "cmd8910: ";
//    cout << frame_cmd[8].toLatin1().data();
//    cout << frame_cmd[9].toLatin1().data();
//    cout << frame_cmd[10].toLatin1().data();
//    cout << endl;
//    cout << "cmd_roll2_x: " << cmd_roll2_x << endl;
//    cout << "cmd_roll2_y: " << cmd_roll2_y << endl;
//    cout << "cmd_roll2_z: " << cmd_roll2_z << endl;
}

/*----------------------------------------*/





void Terminal::cmd111213_save()
{
    cmd_vehicle_theta_old = cmd_vehicle_theta;
    cmd_vehicle_pho_old = cmd_vehicle_pho;
    cmd_vehicle_self_rot_old = cmd_vehicle_self_rot;

    int temp_y = QString2HexInt_single(frame_cmd[10]);
    int temp_x = QString2HexInt_single(frame_cmd[11]);
    int temp_z = QString2HexInt_single(frame_cmd[12]);

    if (flag_x_zero == false)
    {
        flag_x_zero = true;
        VEHICLE_X_ZERO = temp_x;
    }
    if (flag_y_zero == false)
    {
        flag_y_zero = true;
        VEHICLE_Y_ZERO = temp_y;
    }
    if (flag_z_zero == false)
    {
        flag_z_zero = true;
        VEHICLE_Z_ZERO = temp_z;
    }


    double x, y;

    if (temp_x > VEHICLE_X_ZERO)
        x = VEHICLE_RADIUS / (VEHICLE_X_MAX - VEHICLE_X_ZERO) * (temp_x - VEHICLE_X_ZERO);
    else
        x = VEHICLE_RADIUS / (VEHICLE_X_ZERO - VEHICLE_X_MIN) * (temp_x - VEHICLE_X_ZERO);

    if (temp_y > VEHICLE_Y_ZERO)
        y = VEHICLE_RADIUS / (VEHICLE_Y_MAX - VEHICLE_Y_ZERO) * (temp_y - VEHICLE_Y_ZERO);
    else
        y = VEHICLE_RADIUS / (VEHICLE_Y_ZERO - VEHICLE_Y_MIN) * (temp_y - VEHICLE_Y_ZERO);

    cmd_vehicle_theta = atan2(y, x) / pi * 180.0;

    if ((cmd_vehicle_theta >= -45 && cmd_vehicle_theta <= 45)
            || (cmd_vehicle_theta >= 135 && cmd_vehicle_theta <= 180)
            || (cmd_vehicle_theta >= -180 && cmd_vehicle_theta <= -135))
    {
        cmd_vehicle_pho = sqrt((x*x + y*y) / (VEHICLE_RADIUS*VEHICLE_RADIUS + y*y));
    }
    else
    {
        cmd_vehicle_pho = sqrt((x*x + y*y) / (VEHICLE_RADIUS*VEHICLE_RADIUS + x*x));

    }


    if (temp_z > VEHICLE_Z_ZERO)
        cmd_vehicle_self_rot = 1 / (VEHICLE_Z_MAX - VEHICLE_Z_ZERO) * (temp_z - VEHICLE_Z_ZERO);
    else
        cmd_vehicle_self_rot = 1 / (VEHICLE_Z_ZERO - VEHICLE_Z_MIN) * (temp_z - VEHICLE_Z_ZERO);


//        cout << "cmd111213: ";
//        cout << frame_cmd[11].toLatin1().data();
//        cout << frame_cmd[12].toLatin1().data();
//        cout << frame_cmd[13].toLatin1().data();
//        cout << endl;
//        cout << "temp_x: " << temp_x << endl;
//        cout << "temp_y: " << temp_y << endl;
//        cout << "temp_z: " << temp_z << endl;
//        cout << "x: " << x << endl;
//        cout << "y: " << y << endl;
//        cout << "cmd_vehicle_theta: " << cmd_vehicle_theta << endl;
//        cout << "cmd_vehicle_pho: " << cmd_vehicle_pho << endl;
//        cout << "cmd_vehicle_self_rot: " << cmd_vehicle_self_rot << endl;
}

void Terminal::cmd141516_save()
{

    cmd_roll1_x_old = cmd_roll1_x;
    cmd_roll1_y_old = cmd_roll1_y;
    cmd_roll1_z_old = cmd_roll1_z;

    int temp_y = QString2HexInt_single(frame_cmd[13]);
    int temp_x = QString2HexInt_single(frame_cmd[14]);
    int temp_z = QString2HexInt_single(frame_cmd[15]);

    if (flag_x_roll1_zero == false)
    {
        flag_x_roll1_zero = true;
        roll1_X_ZERO = temp_x;
    }
    if (flag_y_roll1_zero == false)
    {
        flag_y_roll1_zero = true;
        roll1_Y_ZERO = temp_y;
    }
    if (flag_z_roll1_zero == false)
    {
        flag_z_roll1_zero = true;
        roll1_Z_ZERO = temp_z;
    }


    if (temp_x > roll1_X_ZERO)
        cmd_roll1_x = 1 / (VEHICLE_X_MAX - roll1_X_ZERO) * (temp_x - roll1_X_ZERO);
    else
        cmd_roll1_x = 1 / (roll1_X_ZERO - VEHICLE_X_MIN) * (temp_x - roll1_X_ZERO);


    if (temp_y > roll1_Y_ZERO)
        cmd_roll1_y = 1 / (VEHICLE_Y_MAX - roll1_Y_ZERO) * (temp_y - roll1_Y_ZERO);
    else
        cmd_roll1_y = 1 / (roll1_Y_ZERO - VEHICLE_Y_MIN) * (temp_y - roll1_Y_ZERO);


    if (temp_z > roll1_Z_ZERO)
        cmd_roll1_z = 1 / (VEHICLE_Z_MAX - roll1_Z_ZERO) * (temp_z - roll1_Z_ZERO);
    else
        cmd_roll1_z = 1 / (roll1_Z_ZERO - VEHICLE_Z_MIN) * (temp_z - roll1_Z_ZERO);



//    cout << "cmd141516: ";
//    cout << frame_cmd[14].toLatin1().data();
//    cout << frame_cmd[15].toLatin1().data();
//    cout << frame_cmd[16].toLatin1().data();
//    cout << endl;
//    cout << "temp_x: " << temp_x << endl;
//    cout << "temp_y: " << temp_y << endl;
//    cout << "temp_z: " << temp_z << endl;
}


string Terminal::get_arm_joint_space_status()
{
    QString str = QString(cmd_arm_forward_1_true)
                + QString(cmd_arm_back_1_false)
                + QString(cmd_arm_left_2_true)
                + QString(cmd_arm_right_2_false)
                + QString(cmd_arm_up_3_true)
                + QString(cmd_arm_down_3_false)
                + QString(cmd_arm_x_true_4_true)
                + QString(cmd_arm_x_false_4_false)
                + QString(cmd_arm_y_true_5_true)
                + QString(cmd_arm_y_false_5_false)
                + QString(cmd_arm_z_true_6_true)
                + QString(cmd_arm_z_false_6_false);
    QByteArray ba = str.toLatin1();
    string data = ba.data();
    return data;
}








