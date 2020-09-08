#include "mainwindow.h"
#include <QApplication>
#include <QProcess>

/* For the start page. */
#include <ctime>
#include <QSplashScreen>
#include <QPixmap>

/* For sleep(). */
#include <unistd.h>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //The start page
    QSplashScreen *splash = new QSplashScreen;
    QPixmap pixmap(":/imgs/robot.jpg");

    splash->setPixmap(pixmap);
    splash->show();
    sleep(1);
    MainWindow w;
    splash->finish(&w);
    w.show();

    int e = a.exec();

    if (e == RETCODE_RESTART){
       QProcess::startDetached(qApp->applicationFilePath(), QStringList());
       return 0;
    }

    return e;
}
