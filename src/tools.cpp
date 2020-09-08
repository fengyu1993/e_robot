//
// Created by m on 11/14/18.
//

#include "tools.h"

Grid::Grid()
{
    step = 0.3;
    half_each_col_nums = 5;
    half_each_row_nums = 5;
    generate_grid();
}

Grid::Grid(int half_row, int half_col, double step):
half_each_row_nums(half_row),
half_each_col_nums(half_col),
step(step)
{
    generate_grid();
}

Grid::~Grid()
{}

void Grid::generate_grid()
{
    generate_grid_north_south();
    generate_grid_west_east();
}

void Grid::generate_grid_north_south()
{
    double north_y = -half_each_row_nums*step;
    double x = -half_each_col_nums*step;
    for (int i = 0; i < half_each_col_nums*2 + 1; ++i, x+=step)
    {
        Vector3d pts_north(x, north_y, 0.);
        Vector3d pts_south(x, -north_y, 0.);
        grid_north.push_back(pts_north);
        grid_south.push_back(pts_south);

    }
}

void Grid::generate_grid_west_east()
{
    double west_x = -half_each_col_nums*step;
    double y = -half_each_col_nums*step;
    for (int i = 0; i < half_each_row_nums*2 + 1; ++i, y+=step)
    {
        Vector3d pts_west(west_x, y, 0.);
        Vector3d pts_east(-west_x, y, 0.);
        grid_west.push_back(pts_west);
        grid_east.push_back(pts_east);

    }
}

/**
 *
 * @param pts_plane
 * @param Rcp
 * @param tcp
 * @param K
 * @return
 */
vector<Vector2d> Grid::project_pts_to_image(const vector<Vector3d> &pts_plane, const Matrix3d &Rcp, const Vector3d &tcp, const Matrix3d &K)
{
    vector<Vector2d> pts_uv;
    for (auto pt:pts_plane)
    {
        Vector2d pt_uv;
        Vector3d pts_cam = Rcp*pt + tcp;
        pts_cam /= pts_cam(2);
        pts_cam(2) = 1.0;
        Vector3d pt_tmp = K*pts_cam;
        pt_uv = Vector2d(pt_tmp(0), pt_tmp(1));
        pts_uv.push_back(pt_uv);
    }
    return pts_uv;
}

vector<Vector3d> project_uv_to_camera(const vector<Point_uvd> &pts_uv, const Matrix3d &K)
{
    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);

    vector<Vector3d> pts_cam;
    for (int i = 0; i < pts_uv.size(); ++i)
    {
        double u = pts_uv[i].point.x;
        double v = pts_uv[i].point.y;
        Vector3d pt_cam;
        pt_cam(0) = (u-cx)/fx * pts_uv[i].depth;
        pt_cam(1) = (v-cy)/fy * pts_uv[i].depth;
        pt_cam(2) = pts_uv[i].depth;
        pts_cam.push_back(pt_cam);
    }

    return pts_cam;

}