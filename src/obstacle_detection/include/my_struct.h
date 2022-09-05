#ifndef _MY_STRUCT_H_
#define _MY_STRUCT_H_
#include <iostream>
#include <string>

using namespace std;


//定义像素坐标uv
struct point_uv
{
    int x;
    int y;
    //解决map关联容器中函数重载问题
    bool operator < (const point_uv& pk) const
    {
        if (this->x < pk.x)
            return true; 
        else if (this->x == pk.x && this->y < pk.y)
            return true;
        return false;
    }
};


//定义雷达坐标xyz
struct point_xyz
{
    float x;
    float y;
    float z;

};



//包围盒尺寸
struct pointcloud
{
	float y;
	float z;
	float x;
	float length;//长
	float heighth;//高
	float depth;//深
	float distance;
};


struct obstacle
{
    std::string name;
    int Address;
    int u_left;
    int v_left;
    int u_right;
    int v_right;

    float length;
    float depth;
    float height;

    float mass_x;
    float mass_y;
    float mass_z;


};

struct order
{
    int Code;
    int control;
    int data_i1;
    int data_i2;
    float data_f1;
    float data_f2;
};


#endif