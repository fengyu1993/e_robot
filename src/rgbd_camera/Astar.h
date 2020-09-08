#pragma once
/*
//A*算法对象类
*/

#include <vector>
#include <list>
#include <stdlib.h>

const int kCost1=10; //直移一格消耗
const int kCost2=14; //斜移一格消耗

struct Pointa
{
    int x,y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F,G,H; //F=G+H
    Pointa *parent; //parent的坐标，这里没有用指针，从而简化代码
    Pointa(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(NULL)  //变量初始化
    {
    }
};


class Astar
{
public:
    void InitAstar(std::vector<std::vector<int>> &_vec);
    std::list<Pointa *> GetPath(Pointa &startPoint,Pointa &endPoint,bool isIgnoreCorner);

private:
    Pointa *findPath(Pointa &startPoint,Pointa &endPoint,bool isIgnoreCorner);
    std::vector<Pointa *> getSurroundPoints(const Pointa *point,bool isIgnoreCorner) const;
    bool isCanreach(const Pointa *point,const Pointa *target,bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
    Pointa *isInList(const std::list<Pointa *> &list,const Pointa *point) const; //判断开启/关闭列表中是否包含某点
    Pointa *getLeastFpoint(); //从开启列表中返回F值最小的节点
    //计算FGH值
    int calcG(Pointa *temp_start,Pointa *point);
    int calcH(Pointa *point,Pointa *end);
    int calcF(Pointa *point);
private:
    std::vector<std::vector<int>> vec;
    std::list<Pointa *> openList;  //开启列表
    std::list<Pointa *> closeList; //关闭列表
};
