#include "serial_common.h"

/**
 * 构造函数 这里主要初始化串口通信类与指令集
 * @param port
 * @param rate
 * @param data_bit
 * @param check_bit
 * @param stop_bit
 */
Serial::Serial(QString port, int rate, int data_bit, int check_bit, int stop_bit)
{
    // checke the rate
    if(rate %1200 !=0)
        qDebug() << "The buadrate is not support!";

    serial = new QSerialPort;
    //set port
    serial->setPortName(port);
    //open serial
    serial->open(QIODevice::ReadWrite);
    if(serial->isOpen())
        cout << "serial success" << endl;
    else
        cout << "serial false" << endl;
    //set buadrate
    serial->setBaudRate(rate);
    //set data_bit
    switch(data_bit)
    {
    case 8: serial->setDataBits(QSerialPort::Data8); break;
    default: break;
    }
    //set_check
    switch(check_bit)
    {
    case 0: serial->setParity(QSerialPort::NoParity); break;
    default: break;
    }
    //set_stop_bit
    switch(stop_bit)
    {
    case 1: serial->setStopBits(QSerialPort::OneStop); break;
    case 2: serial->setStopBits(QSerialPort::TwoStop); break;
    default: break;
    }
    //设置流控制
    serial->setFlowControl(QSerialPort::NoFlowControl);

    init_control_table();  //Tripod head

}


void Serial::disconnect()
{
    if(serial!=nullptr)
    {
        serial->clear();
        serial->close();
        serial->deleteLater();
    }
}



void Serial::init_control_table()
{
    control_info_table.insert(make_pair(TURN_LEFT      ,     "FF 01 00 04 20 20 45"));
    control_info_table.insert(make_pair(TURN_RIGHT     ,     "FF 01 00 02 20 20 43"));
    control_info_table.insert(make_pair(TURN_UP        ,     "FF 01 00 08 20 20 49"));
    control_info_table.insert(make_pair(TURN_DOWN      ,     "FF 01 00 10 20 20 51"));
    control_info_table.insert(make_pair(STOP           ,     "FF 01 00 00 00 00 01"));
    control_info_table.insert(make_pair(TURN_UP_LEFT   ,     "FF 01 00 0C 20 20 4D"));
    control_info_table.insert(make_pair(TURN_UP_RIGHT  ,     "FF 01 00 0A 20 20 4B"));
    control_info_table.insert(make_pair(TURN_DOWN_LEFT ,     "FF 01 00 14 20 20 55"));
    control_info_table.insert(make_pair(TURN_DOWN_RIGHT,     "FF 01 00 12 20 20 53"));
    control_info_table.insert(make_pair(ZOOM_MINUS     ,     "FF 01 00 40 00 00 41"));
    control_info_table.insert(make_pair(ZOOM_PLUS      ,     "FF 01 00 20 00 00 21"));
    control_info_table.insert(make_pair(FOCUS_MINUS    ,     "FF 01 01 00 00 00 02"));
    control_info_table.insert(make_pair(FOCUS_PLUS     ,     "FF 01 08 80 00 00 81"));
    control_info_table.insert(make_pair(APERTURE_ON    ,     "FF 01 02 00 00 00 03"));
    control_info_table.insert(make_pair(APERTURE_OFF   ,     "FF 01 04 00 00 00 05"));
    control_info_table.insert(make_pair(MASK_ON        ,     "FF 01 00 09 00 01 0b"));
    control_info_table.insert(make_pair(MASK_OFF       ,     "FF 01 00 0b 00 01 0d"));
    control_info_table.insert(make_pair(WIPER_ON       ,     "FF 01 00 09 00 02 0c"));
    control_info_table.insert(make_pair(WIPER_OFF      ,     "FF 01 00 0b 00 02 0e"));

}

void Serial::QStringtoHex(QByteArray &sendData, QString str)
{
    char hstr,lstr,hdata,ldata;
    int len = str.length();
    int sendnum = 0;
    QByteArray temp;
    temp.resize(len/2);//设置大小，len/2会大于实际16进制字符
    //sendData.resize(len/2);
    for(int i=0;i<len;)
    {
        //hstr = str[i].toAscii();
        hstr = str[i].toLatin1();
        if(hstr == ' ')
        {
            ++i;
            continue;
        }
        ++i;
        if(i >= len)
        {
            break;
        }
        lstr = str[i].toLatin1();

        hdata = ConvertHexChar(hstr);
        ldata = ConvertHexChar(lstr);
        if(-1 == hdata || -1 == ldata)
        {
            break;
        }
        ++i;
        temp[sendnum] = hdata<<4|ldata;
        sendnum++;
    }
    sendData.reserve(sendnum);
    sendData = temp.left(sendnum);//去掉多余字符
}

char Serial::ConvertHexChar(char c)
{
    if(c>='a'&&c<='f')
    {
        return c-'a'+10;
    }
    else if(c>='A'&&c<='F')
    {
        return c-'A'+10;
    }
    else if(c>='0'&&c<='9')
    {
        return c-'0';
    }
    else{
        return -1;
    }
}


bool Serial::send_data(int key)
{
    if(control_info_table.count(key))
    {
//        qDebug() << control_info_table[key];
        serial_write(control_info_table[key]);
        return true;
    }
    else {
        return false;
    }
}

bool Serial::serial_write(QString &se_data)
{
    QByteArray sendbuff;

    QStringtoHex(sendbuff, se_data);


    serial->write(sendbuff);

    return true;
}

//读取接收到的信息
bool Serial::read_data(QString &re_data)
{
    QByteArray buf;
    buf = serial->readAll();

    QString strHex;

    if(!buf.isEmpty())
    {

      QDataStream out(&buf,QIODevice::ReadWrite);
      while(!out.atEnd())
      {
            qint8 outChar = 0;
            out>>outChar;
            strHex += QString("%1").arg(outChar&0xFF,2,16,QLatin1Char('0'));
      }
      re_data += strHex;
      buf.clear();
      return true;
    }
    else
    {
        buf.clear();
        return false;
    }

}

