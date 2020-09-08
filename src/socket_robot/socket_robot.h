/**
 * 1.SocketRobot主要搭建了一个socket服务，为通信建立起平台
 * 2.MsgInfo msginfo主要是协议，包含校验数据，数据合法性等等
 * 3.
 */
#ifndef SOCKET_ROBOT_H
#define SOCKET_ROBOT_H

#include <QObject>
#include <QMessageBox>
#include <qt5/QtCore/qthread.h>

/* for communication */
#include <QTcpServer>
#include <QTcpSocket>
#include <string>
#include <vector>
#include "utility.h"
#include "msg_info.h"

/*std iostream*/
#include<iostream>
using namespace std;

#define     ide_HtoS_Vehicle_Status         "1001"
#define     ide_HtoS_Target_Posture         "1002"
#define     ide_HtoS_ARM1_Joint_Angle       "1003"
#define     ide_HtoS_Choose_Workspace       "1004"
#define     ide_HtoS_ARM2_Joint_Angle       "1005"

#define     long_HtoS_Vehicle_Status        "05"
#define     long_HtoS_Target_Posture        "48"
#define     long_HtoS_ARM1_Joint_Angle      "48"
#define     long_HtoS_Choose_Workspace      "01"
#define     long_HtoS_ARM2_Joint_Angle      "48"

#define     ide_StoH_ARM1_Joint_Angle       "0001"
#define     ide_StoH_ARM1_Flag_Move_End     "0002"
#define     ide_StoH_ARM1_Actuator_Posture  "0003" // Right Arm
#define     ide_StoH_ARM1_Joint_Status      "0004"
#define     ide_StoH_Vehicle_Speed          "0005"
#define     ide_StoH_ARM2_Joint_Angle       "0006"
#define     ide_StoH_ARM2_Joint_Status      "0007"
#define     ide_StoH_ARM2_Flag_Move_End     "0008"
#define     ide_StoH_ARM2_Actuator_Posture  "0009" // Left Arm
#define     ide_StoH_Error_Code             "0010"

#define     ide_StoH_All_Data               "0099"


#define     long_StoH_ARM1_Joint_Angle      "48"
#define     long_StoH_ARM1_Flag_Move_End    "01"
#define     long_StoH_ARM1_Actuator_Posture "48"
#define     long_StoH_ARM1_Joint_Status     "06"
#define     long_StoH_Vehicle_Speed         "08"
#define     long_StoH_ARM2_Joint_Angle      "48"
#define     long_StoH_ARM2_Joint_Status     "06"
#define     long_StoH_ARM2_Flag_Move_End    "01"
#define     long_StoH_ARM2_Actuator_Posture "48"

#define     ide_Both_Handshake              "2001"
#define     ide_Both_Answer                 "2002"
#define     ide_Both_Host_Ask_Data          "2003"

#define     long_Both_Handshake             "04"
#define     long_Both_Answer                "04"
#define     long_Both_Host_Ask_Data         "04"

#define     rec_success                     "1111"
#define     rec_chkerror                    "2222"
#define     rec_longerror                   "3333"
#define     rec_iderror                     "4444"
#define     rec_handshake                   "1234"

class SocketRobot : public QObject //QThread
{
    Q_OBJECT

public:
    float       StoH_ARM1_Joint_Angle[6];
    bool        StoH_ARM1_Flag_Move_End;
    float       StoH_ARM1_Actuator_Posture[6];
    bool        StoH_ARM1_Joint_Status[6];
    float       StoH_Vehicle_Speed;
    float       StoH_ARM2_Joint_Angle[6];
    bool        StoH_ARM2_Joint_Status[6];
    bool        StoH_ARM2_Flag_Move_End;
    float       StoH_ARM2_Actuator_Posture[6];
    int         StoH_Error_Code;

    string      RecOrderOld;
    string      RecOrder;

    explicit    SocketRobot(QObject *parent = nullptr);
    virtual     ~SocketRobot();
    void        ServerSendData(const string &data);


    Arm_pose    arm1_pose;
    Arm_pose    arm2_pose;


    bool        set_arm1_pose(const vector<int> &arm_pose);
    bool        set_arm2_pose(const vector<int> &arm_pose);
    int         check_recv_data(const string & str);



    bool        send_msg(const string & src);
    string      check_sum(const string& str);
    string      encrypt_msg(const string& str); //加密
    string      decrypt_msg(const string& str);

private Q_SLOTS:
    void        ServerNewConnection();
    void        ServerReadData();
    void        ServerDisConnection();

Q_SIGNALS:
    void        receive_arm1_joint_angle();
    void        receive_arm2_joint_angle();
    void        receive_arm1_joint_status();
    void        receive_arm2_joint_status();
    void        receive_vehicle_speed();
    void        receive_arm1_actuator_posture();
    void        receive_arm2_actuator_posture();
    void        receive_error_code();

    void        receive_all_data();

private:
    QTcpServer  *mp_TCPServer;
    QTcpSocket  *mp_TCPSocket;
    bool        is_client_ready;

    map<string, string> data_header;
    int         mission_id;
    vector<char> key;
    MsgInfo     *msg;
    string      recv_data;

    void        Save_Data_ARM1_Joint_Angle(string Data);
    void        Save_Data_ARM1_Flag_Move_End(string Data);
    void        Save_Data_ARM1_Actuator_Posture(string Data);
    void        Save_Data_ARM1_Joint_Status(string Data);
    void        Save_Data_Vehicle_Speed(string Data);
    void        Save_Data_ARM2_Joint_Angle(string Data);
    void        Save_Data_ARM2_Flag_Move_End(string Data);
    void        Save_Data_ARM2_Actuator_Posture(string Data);
    void        Save_Data_ARM2_Joint_Status(string Data);
    void        Save_Data_Error_Code(string Data);

};






#endif // SOCKET_ROBOT_H
