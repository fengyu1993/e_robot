//
// Created by e on 3/24/19.
//

#ifndef E_ROBOT_MSGINFO_H
#define E_ROBOT_MSGINFO_H

#include <QString>
#include <regex>
#include "utility.h"
#include <QObject>

#define flag_rec_success    1
#define flag_rec_chkerror   2
#define flag_rec_longerror  3
#define flag_rec_iderror    4

#define long_mark           4
#define long_size           4
#define long_check          4
#define long_else           (long_mark + long_size + long_check)

class MsgInfo : public QObject
{
    Q_OBJECT
public:
    explicit MsgInfo(const string & str);
            ~MsgInfo(){}
    void show();
    int is_valid();
    string get_mark();
    string get_main_data();
    string check_sum(const string &str);

private:
    string data;
    string mark;
    string size;
    string main_data;
    string check;
};


//string check_sum(const string &str);

#endif //E_ROBOT_MSGINFO_H
