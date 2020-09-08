#include "ping_check.h"


PingCheck::PingCheck(QString ip)
{
    cmdstr = "ping " + ip;
    checkstr = "64 bytes from " + ip;
}

PingCheck::~PingCheck()
{
    terminate();
}

bool PingCheck::init()
{
    start();
    return true;
}

void PingCheck::run()
{
    QTextCodec *codec = QTextCodec::codecForName("GBK");

    QProcess exc;

    while (true)
    {

        exc.start(cmdstr);

        while (exc.waitForFinished(3000) == false) {

            QByteArray out = exc.readAllStandardOutput();

            if (!out.isEmpty()) {
                if((out.indexOf(checkstr) != -1 )){
//                    std::cout << out.data() << std::endl;
//                    std::cout << "ping success" << std::endl;
                }
                else{
//                    std::cout << "ping fail" << std::endl;
//                    std::cout << out.data() << std::endl;
                    Q_EMIT pingfail();
                }

            }
            else{
                Q_EMIT pingfail();
            }
        }

    }
}


