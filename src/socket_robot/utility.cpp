#include "utility.h"

string Utility::paddingZero(const double &data)
{
    char buffer[80];
    sprintf(buffer,"%08.3f",data);
    string out(buffer);
    return out;
}

string Utility::paddingZero(const int &data)
{
    char buffer[80];
    sprintf(buffer,"%04d",data);
    string out(buffer);
    return out;
}