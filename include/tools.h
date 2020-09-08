//
// Created by m on 11/14/18.
//

#ifndef E_ROBOT_TOOLS_H
#define E_ROBOT_TOOLS_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

using namespace std;
using namespace Eigen;

/*  grid information
 *          north
 *      * * * * * * *
 *      * * * * * * *
 * west * * * * * * * east
 *      * * * * * * *
 *      * * * * * * *
 *          south
 */

/*  plane coor
 *            +y
 *      * * * * * * *
 *      * * * * * * *
 *  -x  * * * * * * *  x+
 *      * * * * * * *
 *      * * * * * * *
 *            -y
 */

class Grid
{
public:
    Grid();
    Grid(int half_row, int half_col, double step);
    ~Grid();


    vector<Vector3d> grid_north;
    vector<Vector3d> grid_south;
    vector<Vector3d> grid_west;
    vector<Vector3d> grid_east;
    void generate_grid_north_south();
    void generate_grid_west_east();
    void generate_grid();
    vector<Vector2d> project_pts_to_image(const vector<Vector3d> &pts_plane, const Matrix3d &Rcp, const Vector3d &tcp, const Matrix3d &K);//像素坐标转换成相机坐标

private:
    double step;
    int half_each_row_nums; //(4+1+4)
    int half_each_col_nums;
};

struct Point_uvd
{
   cv::Point point;
   double depth;
};

vector<Vector3d> project_uv_to_camera(const vector<Point_uvd> &pts_uv, const Matrix3d &K);//相机坐标转换成像素坐标



#endif //E_ROBOT_TOOLS_H
