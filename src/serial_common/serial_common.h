/**
 * 1.这是自己写的一个串口通信库，不单单可以与485协议的云台控制进行通信，如果后续有串口的设备也可以用这个类去实例化一个具体的对象去通信。
 * 2.通用协议的智能云台控制协议为：PELCO-D、PELCO-P, windows有一个软件能够控制云台，并且有发送的指令，我根据之前成功的指令写到了这套代码中，所以建议
 *   修改之前先用windows下的软件测试一下，之后再进行修改。
 * 3.这套代码我并未进行测试实物测试，而且这套代码是根据lixiang之前改过的，但是理论上应该没什么大问题。
 * 4.如果这些代码有用，那它们是xinliang zhong写的，如果没用，那我就不知道是谁写的了。
 *
 * Author: xinliangzhong@foxmail.com
 * Data: 2019.06.10
 */
#ifndef SERIAL_COMMON_H
#define SERIAL_COMMON_H

#include <QDebug>
#include <QErrorMessage>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <map>
#include <unordered_map>
#include <iostream>

using namespace std;


/**
 *  TURN_LEFT         "FF 01 00 04 20 20 45"
 *  TURN_RIGHT        "FF 01 00 02 20 20 43"
 *  TURN_UP           "FF 01 00 08 20 20 49"
 *  TURN_DOWN         "FF 01 00 10 20 20 51"
 *  STOP              "FF 01 00 00 00 00 01"
 *  TURN_UP_LEFT      "FF 01 00 0C 20 20 4D"
 *  TURN_UP_RIGHT     "FF 01 00 0A 20 20 4B"
 *  TURN_DOWN_LEFT    "FF 01 00 14 20 20 55"
 *  TURN_DOWN_RIGHT   "FF 01 00 12 20 20 53"
 *  ZOOM_MINUS        "FF 01 00 40 00 00 41"
 *  ZOOM_PLUS         "FF 01 00 20 00 00 21"
 *  FOCUS_MINUS       "FF 01 01 00 00 00 02"
 *  FOCUS_PLUS        "FF 01 08 80 00 00 81"
 *  APERTURE_ON       "FF 01 02 00 00 00 03"
 *  APERTURE_OFF      "FF 01 04 00 00 00 05"
 *  MASK_ON           "FF 01 00 09 00 01 0b"
 *  MASK_OFF          "FF 01 00 0b 00 01 0d"
 *  WIPER_ON          "FF 01 00 09 00 02 0c"
 *  WIPER_OFF         "FF 01 00 0b 00 02 0e"
 */

enum GLOBAL_CONTROL_INFO
{
    FOCUS_PLUS = 0,
    FOCUS_MINUS,
    ZOOM_PLUS,
    ZOOM_MINUS,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_UP,
    TURN_DOWN,
    TURN_UP_LEFT,
    TURN_UP_RIGHT,
    TURN_DOWN_LEFT,
    TURN_DOWN_RIGHT,
    STOP,
    ORIGIN,
    APERTURE_ON,
    APERTURE_OFF,
    MASK_ON,
    MASK_OFF,
    WIPER_ON,
    WIPER_OFF
};





class Serial
{
public:
    explicit Serial(QString port, int rate,
                    int data_bit = 8, int check_bit = 0, int stop_bit =1); //通信参数：端口，波特率，数据位，校验位，停止位
    ~Serial();
    void disconnect();

    void init_control_table(); //初始化指令表，如果有指令在这个函数里边加进去

    void QStringtoHex(QByteArray &sendData, QString str);
    char ConvertHexChar(char c);

    /**
     * 发送指令的两个函数,send_data()为外部接口.比如对应于界面上不同按键的响应;
     * serial_write()为内部函数，串口write函数
     */
    bool send_data(int key);
    bool serial_write(QString &data);
    bool read_data(QString &data);
    QSerialPort *serial;    //serial

private:
    //通信参数：端口，波特率，数据位，校验位，停止位
    QString port;
    int rate;
    int data_bit;
    int check_bit;
    int stop_bit;
    map<int,QString> control_info_table; // 指令表


};

#endif // SERIAL_COMMON_H

