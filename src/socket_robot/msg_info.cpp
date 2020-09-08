//
// Created by e on 3/24/19.
//

#include "msg_info.h"
#include "utility.h"
#include "regex.h"

using namespace std;

MsgInfo::MsgInfo(const string &str):data(str)
{
    int len = data.size();
    mark = data.substr(0,long_mark); //4
    size = data.substr(long_mark,long_size); //4
    check = data.substr(len-long_check,long_check); //4
    main_data = data.substr(long_mark + long_size,len-long_else);
}

void MsgInfo::show() {
    cout << "The msg is:\n" << endl;
    cout << "      mark: " << mark << endl;
    cout << "      size: " << size << endl;
    cout << " main_data: " << main_data << endl;
    cout << "     check: " << check << endl;

}

int MsgInfo::is_valid() {
//    regex reg_mark("[0-9]{long_mark}");
//    regex reg_size("[0-9]{long_size}");
//    regex reg_check("[0-9]{long_check}");

    regex reg_mark("[0-9]{4}");
    regex reg_size("[0-9]{4}");
    regex reg_check("[0-9]{4}");

    int main_data_size = Utility::string2<int>(size);
    int len = data.size() - long_else;
    cout << "data.size(): " << data.size() << endl;
    string chk_cal = data.substr(0, data.size()-long_check);

    if (regex_match(mark,reg_mark) && regex_match(size,reg_size) && regex_match(check,reg_check))
    {
       if ( main_data_size != len )
           return flag_rec_longerror;
       else if ( check_sum(chk_cal) != check )
           return flag_rec_chkerror;
       else
           return flag_rec_success;
    }
    else
    {
        return flag_rec_iderror;
    }

}
string MsgInfo::get_mark()
{
    return mark;
}

string MsgInfo::get_main_data()
{
    return main_data;
}

string MsgInfo::check_sum(const string &str) {
    int sum = 0;
    const char * data = str.c_str();
    int len = str.size();
    for (int i = 0; i < len; ++i) {
        sum+=data[i]%2;
    }

    return Utility::paddingZero(sum);
}
