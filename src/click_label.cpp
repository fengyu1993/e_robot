#include "click_label.h"
#include <QMouseEvent>
#include<iostream>
using namespace std;

CLick_Label::CLick_Label(QWidget *parent):
    QLabel(parent)
{
    QPalette pa;
    pa.setColor(QPalette::WindowText, Qt::blue);
    setPalette(pa);
    can_move = true;
    pos_x = this->x();
    pos_y = this->y();

    QAction *openAct = new QAction("DAKAI", this);

    connect(openAct, SIGNAL(triggered()), this, SLOT(just_for_test()));
}

CLick_Label::CLick_Label(const QString &text, QWidget *parent):
    QLabel(parent)
{
    QPalette pa;
    pa.setColor(QPalette::WindowText, Qt::blue);
    setPalette(pa);
    setText(text);
    can_move = true;
    pos_x = this->x();
    pos_y = this->y();
}

CLick_Label::~CLick_Label()
{

}

void CLick_Label::setMovable(bool can_move)
{
    this->can_move = can_move;
}

void CLick_Label::mousePressEvent(QMouseEvent *ev)
{
    if(ev->button() == Qt::LeftButton)
    {
        click_x = ev->x();
        click_y = ev->y();
    }
}

void CLick_Label::mouseReleaseEvent(QMouseEvent *ev)
{
    if (ev->button() == Qt::LeftButton)
    {
        release_x = ev->x();
        release_y = ev->y();
        int dx = click_x-release_x;
        int dy = click_y-release_y;
        if(can_move)
            this->move(this->x()-dx, this->y()-dy);
        Q_EMIT clicked();//(this);
    }


}

void CLick_Label::mouseMoveEvent(QMouseEvent *event)
{
    int x = event->x();
    int y = event->y();
    Q_EMIT moved_send_position(x, y);
    if(event->buttons()&Qt::LeftButton)
    {
        cout << "pushLeftButtonAndMove!"<< endl;
//        this->move(event->x(), event->y());
        cout << "old:" << this->x() << " " << this->y() << endl;
        cout << "x:" << x << " y:" << y << endl;
    }else if(event->buttons()&Qt::RightButton)
    {
        cout << "pushRightButtonAndMove!" << endl;
    }
}

void CLick_Label::enterEvent(QEvent *event)
{
    QPalette pa;
    pa.setColor(QPalette::WindowText, Qt::red);
    setPalette(pa);
}

void CLick_Label::leaveEvent(QEvent *)
{
    QPalette pa;
    pa.setColor(QPalette::WindowText, Qt::blue);
    setPalette(pa);
}

void CLick_Label::just_for_test()
{
    cout << "right buttom clicked" << endl;
}
