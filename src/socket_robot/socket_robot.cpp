#include "socket_robot.h"
#include "msg_info.h"
#include "mainwindow.h"

SocketRobot::SocketRobot(QObject *parent)
{

    mp_TCPSocket = nullptr;
    mp_TCPServer = new QTcpServer(this);
    connect(mp_TCPServer, SIGNAL(newConnection()), this, SLOT(ServerNewConnection()));
    mp_TCPServer->listen(QHostAddress::AnyIPv4, 5550);
    is_client_ready = false;

    /*数据包头-4位标识符+2位数据长度+数据+校验长度*/
    /*key 4位标识符, value: 2位数据长度*/
//    data_header[ide_HtoS_Vehicle_Status] = long_HtoS_Vehicle_Status;
//    data_header[ide_HtoS_Target_Posture] = long_HtoS_Target_Posture;
//    data_header[ide_HtoS_ARM1_Joint_Angle] = long_HtoS_ARM1_Joint_Angle;
//    data_header[ide_HtoS_Choose_Workspace] = long_HtoS_Choose_Workspace;
//    data_header[ide_HtoS_ARM2_Joint_Angle] = long_HtoS_ARM2_Joint_Angle;

//    data_header[ide_StoH_ARM1_Joint_Angle] = long_StoH_ARM1_Joint_Angle;
//    data_header[ide_StoH_ARM1_Flag_Move_End] = long_StoH_ARM1_Flag_Move_End;
//    data_header[ide_StoH_ARM1_Actuator_Posture] = long_StoH_ARM1_Actuator_Posture;
//    data_header[ide_StoH_ARM1_Joint_Status] = long_StoH_ARM1_Joint_Status;
//    data_header[ide_StoH_Vehicle_Speed] = long_StoH_Vehicle_Speed;
//    data_header[ide_StoH_ARM2_Joint_Angle] = long_StoH_ARM2_Joint_Angle;
//    data_header[ide_StoH_ARM2_Joint_Status] = long_StoH_ARM2_Joint_Status;
//    data_header[ide_StoH_ARM2_Flag_Move_End] = long_StoH_ARM2_Flag_Move_End;
//    data_header[ide_StoH_ARM2_Actuator_Posture] = long_StoH_ARM2_Actuator_Posture;

//    data_header[ide_Both_Handshake] = long_Both_Handshake;
//    data_header[ide_Both_Answer] = long_Both_Answer;
//    data_header[ide_Both_Host_Ask_Data] = long_Both_Host_Ask_Data;

    mission_id = -1;

    /*key*/
    key = vector<char>{'1','2','3','4','5','6','7'};
    cout << key[0] << endl;

}

SocketRobot::~SocketRobot()
{

}


bool SocketRobot::set_arm1_pose(const vector<int> &arm_pose)
{
    if(arm_pose.size()!=6)
        return false;
    arm1_pose.joint1 = arm_pose[0];
    arm1_pose.joint2 = arm_pose[1];
    arm1_pose.joint3 = arm_pose[2];
    arm1_pose.joint4 = arm_pose[3];
    arm1_pose.joint5 = arm_pose[4];
    arm1_pose.joint6 = arm_pose[5];
}

bool SocketRobot::set_arm2_pose(const vector<int> &arm_pose)
{
    if(arm_pose.size()!=6)
        return false;
    arm2_pose.joint1 = arm_pose[0];
    arm2_pose.joint2 = arm_pose[1];
    arm2_pose.joint3 = arm_pose[2];
    arm2_pose.joint4 = arm_pose[3];
    arm2_pose.joint5 = arm_pose[4];
    arm2_pose.joint6 = arm_pose[5];
}

int SocketRobot::check_recv_data(const string &str)
{

    string dst = decrypt_msg(str);
    msg = new MsgInfo(dst);
    int flag = msg->is_valid();

    return flag;

}

bool SocketRobot::send_msg(const string &src) {

    string check = check_sum(src);
//    cout << "总和校验: " << check << endl;
    check =src + check;
//    cout << "全文: " << check << endl;
    string dst = encrypt_msg(check);
//    cout << "加密 " << dst << endl;
    ServerSendData(dst);
//    cout << "解密 " << decrypt_msg(dst) << endl;
    return true;
}

string SocketRobot::check_sum(const string &str) {
    int sum = 0;
    const char * data = str.c_str();
    int len = str.size();
    for (int i = 0; i < len; ++i) {
        sum+=data[i]%2;
    }

    return Utility::paddingZero(sum);
}

//string SocketRobot::encrypt_msg(const string &str) {
//    int len = str.size();
//    string dst = str;
//    for (int i = 0; i < len; i++) {
//        dst[i] = str[i] ^ key[i % 7];
//    }
//    return dst;
//}

string SocketRobot::encrypt_msg(const string &str) {
    int len = str.size();
    string dst = str;
    int step = len/2;
    for (int i = 0; i < step ; i++) {
        swap(dst[2*i],dst[2*i+1]);
    }
    return dst;
}

string SocketRobot::decrypt_msg(const string &str) {
    int len = str.size();
    string dst = str;
//    for (int i = 0; i < len; i++) {
//        dst[i] = str[i] ^ key[i % 7];
//    }
    int step = len/2;
    for (int i = 0; i < step ; i++) {
        swap(dst[2*i],dst[2*i+1]);
    }
    return dst;
}


void SocketRobot::ServerSendData(const string &data)
{
    char sendMsgChar[1024] = {0};
    QString sendMsg = QString::fromStdString(data);
    cout << "ready to send data" << endl;
    if(sendMsg.isEmpty())
    {
        QMessageBox::information(nullptr, "QT网络通信", "发送数据为空，请输入数据");
        return;
    }
    if(!is_client_ready)
    {
        //QMessageBox::information(nullptr, "QT网络通信", "客户端没有连接！");
        cout << "QT网络通信客户端没有连接！" << endl;
        return;
    }
    strcpy(sendMsgChar, sendMsg.toStdString().c_str());
    if(mp_TCPSocket->isValid())
    {

        int sendRe = mp_TCPSocket->write(sendMsgChar, strlen(sendMsgChar));
        if( -1 == sendRe)
        {
            QMessageBox::information(nullptr, "QT网络通信", "服务端发送数据失败！");
        }
    }
    else
    {
        QMessageBox::information(nullptr, "QT网络通信", "套接字无效！");
    }

}



void SocketRobot::ServerNewConnection()
{
    cout << "New Connection!" << endl;
    mp_TCPSocket = mp_TCPServer->nextPendingConnection();
    if(!mp_TCPSocket)
    {
        QMessageBox::information(nullptr, "QT网络通信", "未正确获取客户端连接！");
        return;
    }
    else
    {
        is_client_ready = true;
        //QMessageBox::information(nullptr, "QT网络通信", "成功接受客户端的连接");
        cout << "成功接受客户端的连接" << endl;
        connect(mp_TCPSocket, SIGNAL(readyRead()), this, SLOT(ServerReadData()));
        connect(mp_TCPSocket, SIGNAL(disconnected()), this, SLOT(ServerDisConnection()));
    }
}

void SocketRobot::ServerReadData()
{
    char buffer[1024] = {0};
    mp_TCPSocket->read(buffer, 1024);
    if( strlen(buffer) > 0)
    {
        QString showMsg = buffer;
        int flag = check_recv_data(showMsg.toStdString());
        bool bResponse = false;
        if(flag == flag_rec_success)
        {
            if (msg->get_mark() == ide_StoH_ARM1_Joint_Angle)
            {
                Save_Data_ARM1_Joint_Angle(msg->get_main_data());
                bResponse = true;
                Q_EMIT receive_arm1_joint_angle();
            }
            else if (msg->get_mark() == ide_StoH_ARM1_Flag_Move_End)
            {
                Save_Data_ARM1_Flag_Move_End(msg->get_main_data());
                bResponse = true;
            }
            else if (msg->get_mark() == ide_StoH_ARM1_Actuator_Posture)
            {
                Save_Data_ARM1_Actuator_Posture(msg->get_main_data());
                bResponse = true;
                Q_EMIT receive_arm1_actuator_posture();
            }
            else if (msg->get_mark() == ide_StoH_ARM1_Joint_Status)
            {
                Save_Data_ARM1_Joint_Status(msg->get_main_data());
                bResponse = true;
                Q_EMIT receive_arm1_joint_status();
            }
            else if (msg->get_mark() == ide_StoH_Vehicle_Speed)
            {
                Save_Data_Vehicle_Speed(msg->get_main_data());
                bResponse = true;
                Q_EMIT receive_vehicle_speed();
            }
            else if (msg->get_mark() == ide_StoH_ARM2_Joint_Angle)
            {
                Save_Data_ARM2_Joint_Angle(msg->get_main_data());
                bResponse = true;
                Q_EMIT receive_arm2_joint_angle();
            }
            else if (msg->get_mark() == ide_StoH_ARM2_Joint_Status)
            {
                Save_Data_ARM2_Joint_Status(msg->get_main_data());
                bResponse = true;
                Q_EMIT receive_arm2_joint_status();
            }
            else if (msg->get_mark() == ide_StoH_ARM2_Flag_Move_End)
            {
                Save_Data_ARM2_Flag_Move_End(msg->get_main_data());
                bResponse = true;
            }
            else if (msg->get_mark() == ide_StoH_ARM2_Actuator_Posture)
            {
                Save_Data_ARM2_Actuator_Posture(msg->get_main_data());
                bResponse = true;
                Q_EMIT receive_arm2_actuator_posture();
            }
            else if(msg->get_mark() == ide_StoH_Error_Code)
            {
                Save_Data_Error_Code(msg->get_main_data());
                cout << msg->get_main_data() << endl;
                bResponse = true;
                Q_EMIT receive_error_code();
            }
            else if (msg->get_mark() == ide_StoH_All_Data)
            {
                string Arm1_Angle   =   msg->get_main_data().substr(0,48);
                string Arm1_Status  =   msg->get_main_data().substr(48,6);
                string Arm1_Posture =   msg->get_main_data().substr(54,48);
                string Arm2_Angle   =   msg->get_main_data().substr(102,48);
                string Arm2_Status  =   msg->get_main_data().substr(150,6);
                string Arm2_Posture =   msg->get_main_data().substr(156,48);
                string Vehicle_Speed =  msg->get_main_data().substr(204,8);

                Save_Data_ARM1_Joint_Angle(Arm1_Angle);
                Save_Data_ARM1_Joint_Status(Arm1_Status);
                Save_Data_ARM1_Actuator_Posture(Arm1_Posture);
                Save_Data_ARM2_Joint_Angle(Arm2_Angle);
                Save_Data_ARM2_Joint_Status(Arm2_Status);
                Save_Data_ARM2_Actuator_Posture(Arm2_Posture);
                Save_Data_Vehicle_Speed(Vehicle_Speed);

                bResponse = true;
                Q_EMIT receive_all_data();
            }
            else if (msg->get_mark() == ide_Both_Handshake)
            {
                RecOrderOld = RecOrder;
                RecOrder = msg->get_main_data().substr(0,4);
                bResponse = true;
            }
            else if (msg->get_mark() == ide_Both_Answer)
            {
                RecOrderOld = RecOrder;
                RecOrder = msg->get_main_data().substr(0,4);
                bResponse = false;
            }
            else
            {
                flag = flag_rec_iderror;
                bResponse = true;
            }
        }

        if(bResponse)
        {
            if(flag == flag_rec_success)
            {
                if(RecOrder == rec_handshake)
                {
                    string msg_info = Utility::toString<int>(Both_Handshake) + rec_handshake;
                    cout << "标识符+长度: " << Both_Handshake << endl;
                    cout << "     data: " << rec_handshake << endl;
                    send_msg(msg_info);
                }
                else if(RecOrder == rec_chkerror || RecOrder == rec_longerror || RecOrder == rec_iderror )
                {
                    QMessageBox::information(nullptr, "QT网络通信", "The data sent by the host is error");
                }
                else
                {
                    string msg_info = Utility::toString<int>(Both_Answer) + rec_success;
                    cout << "标识符+长度: " << Both_Answer << endl;
                    cout << "     data: " << rec_success << endl;
                    send_msg(msg_info);
                }

                cout << "recieve success" << endl;
            }
            else if (flag == flag_rec_chkerror)
            {
                string msg_info = Utility::toString<int>(Both_Answer) + rec_chkerror;
                cout << "标识符+长度: " << Both_Answer << endl;
                cout << "     data: " << rec_chkerror << endl;
                send_msg(msg_info);
                cout << "check code error" << endl;
                QMessageBox::information(nullptr, "QT网络通信", "Receive: check code error");
            }
            else if (flag == flag_rec_longerror)
            {
                string msg_info = Utility::toString<int>(Both_Answer) + rec_longerror;
                cout << "标识符+长度: " << Both_Answer << endl;
                cout << "     data: " << rec_longerror << endl;
                send_msg(msg_info);
                cout << "long error" << endl;
                QMessageBox::information(nullptr, "QT网络通信", "Receive: long error");
            }
            else if (flag == flag_rec_iderror)
            {
                string msg_info = Utility::toString<int>(Both_Answer) + rec_iderror;
                cout << "标识符+长度: " << Both_Answer << endl;
                cout << "     data: " << rec_iderror << endl;
                send_msg(msg_info);
                cout << "identifier error" << endl;
                QMessageBox::information(nullptr, "QT网络通信", "Receive: identifier error");
            }
        }
    }
    else
    {
        //QMessageBox::information(nullptr, "QT网络通信", "未正确接收数据");
        cout << "QT网络通信未正确接收数据" << endl;
        return;
    }
}

void SocketRobot::ServerDisConnection()
{
    //QMessageBox::information(nullptr, "QT网络通信", "与客户端链接断开");
    cout << "QT网络通信与客户端链接断开" << endl;
}

void SocketRobot::Save_Data_ARM1_Joint_Angle(string Data)
{
    for (int i = 0; i<6; i++ )
    {
        StoH_ARM1_Joint_Angle[i] = atof(Data.substr(i*8,8).c_str()) - 180;
    }
}

void SocketRobot::Save_Data_ARM1_Flag_Move_End(string Data)
{
    StoH_ARM1_Flag_Move_End = atoi(Data.substr(0,1).c_str());
}

void SocketRobot::Save_Data_ARM1_Actuator_Posture(string Data)
{
    for (int i = 0; i<6; i++ )
    {
        if (i < 3)
        {
            StoH_ARM1_Actuator_Posture[i] = atof(Data.substr(i*8,8).c_str()) - 1500;
        }
        else
            StoH_ARM1_Actuator_Posture[i] = atof(Data.substr(i*8,8).c_str()) - 180;
    }
}

void SocketRobot::Save_Data_ARM1_Joint_Status(string Data)
{
    for (int i = 0; i<6; i++ )
    {
        StoH_ARM1_Joint_Status[i] = atoi(Data.substr(i,1).c_str());
    }
}

void SocketRobot::Save_Data_Vehicle_Speed(string Data)
{
    StoH_Vehicle_Speed = atof(Data.substr(0,8).c_str()) - 5;
}

void SocketRobot::Save_Data_ARM2_Joint_Angle(string Data)
{
    for (int i = 0; i<6; i++ )
    {
        StoH_ARM2_Joint_Angle[i] = atof(Data.substr(i*8,8).c_str()) - 180;
    }
}

void SocketRobot::Save_Data_ARM2_Flag_Move_End(string Data)
{
    StoH_ARM2_Flag_Move_End = atoi(Data.substr(0,1).c_str());
}

void SocketRobot::Save_Data_ARM2_Actuator_Posture(string Data)
{
    for (int i = 0; i<6; i++ )
    {
        if (i < 3)
            StoH_ARM2_Actuator_Posture[i] = atof(Data.substr(i*8,8).c_str()) - 1500;
        else
            StoH_ARM2_Actuator_Posture[i] = atof(Data.substr(i*8,8).c_str()) - 180;
    }
}

void SocketRobot::Save_Data_ARM2_Joint_Status(string Data)
{
    for (int i = 0; i<6; i++ )
    {
        StoH_ARM2_Joint_Status[i] = atoi(Data.substr(i,1).c_str());
    }
}

void SocketRobot::Save_Data_Error_Code(string Data)
{
    StoH_Error_Code = atoi(Data.substr(0,8).c_str());
}




