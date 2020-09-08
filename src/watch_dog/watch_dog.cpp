#include "watch_dog.h"

#define time_out 5


UIWatchDog::UIWatchDog(QObject *parent)
{
    m_pTimer = new QTimer(this);

    QObject::connect(m_pTimer, &QTimer::timeout, [this]() {
        m_waitCondition.notify_all();
    });
}

UIWatchDog::~UIWatchDog()
{
    delete m_pTimer;
}

void UIWatchDog::startWatchDog()
{
//    qInfoScope(0);
    m_pTimer->start((time_out - 1) * 1000);
    start_flag = true;

    std::thread([this](){
        while (start_flag)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            auto func = [&](){
                //计算时间避免ntp对时影响
                bool bTimeOut = false;
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<int> time_span = std::chrono::duration_cast<std::chrono::duration<int>>(t2 - t1);

                if(time_span.count() >= time_out)
                {
                    bTimeOut = true;
                }

                return bTimeOut;
            };

            std::cv_status timeStatus = std::cv_status::no_timeout;
            {
                std::unique_lock<std::mutex> lck(m_lock);
                //func 返回false继续等待
                do
                {
                    timeStatus = m_waitCondition.wait_for(lck, std::chrono::milliseconds(time_out * 1000));
                    if(timeStatus != std::cv_status::timeout)
                    {
                        break;
                    }
                }while(!func());
            }


            //主线程卡住，则重启
            if(timeStatus == std::cv_status::timeout)
            {
                try {
                    //重启操作
                    qApp->exit(RETCODE_RESTART);
                } catch (std::exception &e) {
                        std::cout << e.what() << std::endl;
                }
            }
            else
            {
                //qInfo() << "UI is not blocked!";
            }
        }
    }).detach();
}

