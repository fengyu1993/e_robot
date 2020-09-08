

#ifndef E_ROBOT_WATCHDAG_H
#define E_ROBOT_WATCHDAG_H

#include <qt5/QtCore/qthread.h>
#include <iostream>
#include <string>
#include <thread>
#include <QTimer>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <QApplication>

static const int RETCODE_RESTART = 773;

class UIWatchDog:public QThread
{
    Q_OBJECT

 public:
    UIWatchDog(QObject *parent);
    ~UIWatchDog();

    void startWatchDog();

    QTimer *m_pTimer;
    std::condition_variable m_waitCondition;
    std::mutex m_lock;
    bool start_flag;
};





#endif
