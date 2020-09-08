#ifndef E_ROBOT_PING_CHECK_H
#define E_ROBOT_PING_CHECK_H

#include <qt5/QtCore/qthread.h>
#include <iostream>
#include <string>
#include <thread>
#include <QProcess>
#include <QTextCodec>

class PingCheck:public QThread
{
    Q_OBJECT

 public:
    PingCheck(QString ip);

    virtual ~PingCheck();

    void run();

    bool stop_run();

    bool init();

    QString cmdstr;

    QString checkstr;

Q_SIGNALS:
    void pingfail();

};





#endif
