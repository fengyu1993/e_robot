#ifndef UTILITY_H
#define UTILITY_H

#include<QString>
#include<string>
#include<iostream>

#include"cstdio"
#include <vector>
#include <sstream>

using namespace std;



class Utility
{
public:
    static string paddingZero(const double &data);
    static string paddingZero(const int &data);
    template <class T>
        static T string2(const string &str)
        {
            T out;
            stringstream ss;
            ss << str;
            ss >> out;
            return out;
        }
        template <typename T>
        static string toString(const T &val)
        {
            string out;
            stringstream ss;
            ss << val;
            ss >> out;
            return out;
        }

};

struct Arm_pose
{
    Arm_pose(double val1=0, double val2=0, double val3=0, double val4=0, double val5=0, double val6=0):
            joint1(val1),joint2(val2), joint3(val3), joint4(val4), joint5(val5), joint6(val6)
    {}

    Arm_pose(const vector<float> &vals)
    {
        Q_ASSERT(vals.size() == 6);
        joint1 = vals[0];
        joint2 = vals[1];
        joint3 = vals[2];
        joint4 = vals[3];
        joint5 = vals[4];
        joint6 = vals[5];
    }

    ~Arm_pose(){}

    friend Arm_pose operator-(const Arm_pose& target, const Arm_pose& source )
    {
        double v1 = target.joint1-source.joint1;
        double v2 = target.joint2-source.joint2;
        double v3 = target.joint3-source.joint3;
        double v4 = target.joint4-source.joint4;
        double v5 = target.joint5-source.joint5;
        double v6 = target.joint6-source.joint6;
        return Arm_pose(v1,v2,v3,v4,v5,v6);

    }

    string toStdString()
    {
        string v1 = Utility::paddingZero(joint1);
        string v2 = Utility::paddingZero(joint2);
        string v3 = Utility::paddingZero(joint3);
        string v4 = Utility::paddingZero(joint4);
        string v5 = Utility::paddingZero(joint5);
        string v6 = Utility::paddingZero(joint6);
        return v1+v2+v3+v4+v5+v6;
    }

    double joint1;
    double joint2;
    double joint3;
    double joint4;
    double joint5;
    double joint6;
};

#endif // UTILITY_H

