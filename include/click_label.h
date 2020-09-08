#ifndef CLICK_LEBEL_H
#define CLICK_LEBEL_H

#include <QLabel>
#include<QAction>

class CLick_Label:public QLabel
{
    Q_OBJECT

public:
    explicit  CLick_Label(QWidget *parent = 0);
    CLick_Label(const QString &text, QWidget *parent=0);
    ~CLick_Label();
    void setMovable(bool can_move);

private Q_SLOTS:
    void just_for_test();

Q_SIGNALS:
    void clicked();
    void moved_send_position(int x, int y);

protected:
    // 鼠标单击事件
    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);
    void enterEvent(QEvent *event);
    void leaveEvent(QEvent *);
private:
    int click_x, click_y, release_x, release_y;
    bool can_move;
    int pos_x, pos_y;
};

#endif // CLICK_LEBEL_H
