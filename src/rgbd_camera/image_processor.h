/**
 * 1.这个文件主要是针对深度相机realsense D435的代码, 暂时只完成了一个相机的驱动，未来如果多传感器需要实现驱动多个
 * 2.在使用之前先插上摄像头，不支持代码运行之后再插上并显示新插入的相机图像
 * 3.代码相对简单。基本上也就是opencv采集视频流
 * 4.如果这些代码有用，那它们是xinliang zhong写的，如果没用，那我就不知道是谁写的了。
 * 5.如果在一台新的电脑上要使用realsense的SDK，请仔细阅读intel realsense官方给出的安装建议及其使用步骤，
 *   另外给出了很多样例，其中就包括深度图对齐，多相机驱动等等。链接：https://github.com/IntelRealSense/librealsense
 *   此台NUC （epc）已经安装好了.
 * Author: xinliangzhong@foxmail.com
 * Data: 2019.06.10
 */
#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#define RGBD_L_ID   "941322071300"  //车体
#define RGBD_R_ID   "941322071243"  //右机械臂
#define RGBD_C_ID   "018322071530"  //左机械臂


#include <qt5/QtCore/qthread.h>
#include <iostream>
#include <string>
#include <atomic>
#include <map>
#include <thread>
using namespace std;

/*opencv*/
#include <opencv2/opencv.hpp>

/*Eigen*/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>

/*Realsense*/
#include <librealsense2/rs.hpp>
using namespace rs2;
#include "tools.h"

#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace Eigen;

/**
 * 相机参数,realsense 435 可以通过SDK读取
 */
struct CameraInfo
{
    double fx;
    double fy;
    double cx;
    double cy;
    Matrix3d K;
};

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}//结构体的构造函数。把x赋值给x_，把y赋值给y_。也就是说在建立一个对象的时候x_和y_是赋完值的。

    template <typename T>
    bool operator() (
        const T* const a,
         const T* const b,
            const T* const c,
            const T* const d,
            const T* const e,
        T* residual ) const
    {
        //residual[0] = T ( _y ) - (a[0]*exp(b[0]*T ( _x )) + c[0]*exp(d[0]*T ( _x ))); // y-exp(ax^2+bx+c)
        residual[0] = T ( _y ) - (a[0]*pow(T ( _x ),4) + b[0]*pow(T ( _x ),3)+c[0]*pow(T ( _x ),2)+d[0]*pow(T ( _x ),1)+e[0]); // y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;
};





class ImageProcessor : public QThread
{
    Q_OBJECT

public:
    map<QString,int> rgbd_id_list; // 指令表
    bool expShow;

    ImageProcessor();
    virtual ~ImageProcessor();
    bool init();
    void run();
    bool stop();

    QImage toQImage(const cv::Mat &mat);
    cv::Mat getImageColor();
    cv::Mat getImageDepth();
    rs2::frame getDepth();
    cv::Mat getImageColorizedDepth();
    float get_depth_scale(rs2::device dev);

    cv::Mat toOCCImage(const rs2::frameset& framesets);
    rs2::depth_frame toOCDImage(const rs2::frameset& framesets);
    rs2::depth_frame toOCDImage_Show3D(const rs2::frameset& framesets);
    cv::Mat depth_RStoCV(const rs2::depth_frame& depth);
    cv::Mat color_RStoCV(const rs2::video_frame& color);
    rs2::depth_frame depth_filter(const rs2::depth_frame depth);
    cv::Mat getImageColor1();
    cv::Mat getImageDepth1();
    cv::Mat getImageColorizedDepth1();
    cv::Mat getImageColor2();
    cv::Mat getImageDepth2();
    cv::Mat getImageColorizedDepth2();
    int  realsense_numb();


        cv::Mat color_for_plane, depth_for_plane;
        Grid grid;
        bool is_set_plane_img;
        static void on_mouse(int event,int x,int y,int flags,void *ustc);
        void on_mouse(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth);
    void setGroundPlaneEstimateFlag(bool is_estimate = false);
    void estimateGroundPlane();
    pair<Vector3d, Vector3d> estimatePlane(const vector<Vector3d> &pts_uvd);
    void showGroundPlane(pair<Vector3d,Vector3d> &centroid_normal, cv::Mat &img);
    void setRealsenseLaunchFlag(bool is_launch = false);
    std::pair<Vector3d, Vector3d> best_plane_from_points(const std::vector<Vector3d> & c);


    static void on_trackbar(int x,  void *ustc);
    void on_trackbar();
    static void on_mouse_way_points(int event, int x, int y, int flags, void *ustc);
    void on_mouse_way_points(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth);

    void setWayPoints();
    vector<Vector2d> obtain_navigation_waypoints( );
    Vector2d pixel2Camera_x_z(int u, double depth);
    cv::Mat color_for_nav, depth_for_nav;






    void autoWayPoints();
    static void on_auto_way_points(int event, int x, int y, int flags, void *ustc);
    void on_auto_way_points(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth);
    void on_trackbar_auto();
    static  void on_trackbar_auto(int x, void*ustc);

    vector<Vector2d> obtain_auto_navigation_waypoints( );

    cv::Mat color_for_auto, depth_for_auto;
    cv::Mat img_auto;



void setgraspPoints();

static void on_mouse_grasp_points(int event, int x, int y, int flags, void *ustc);
void on_mouse_grasp_points(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth);
cv::Mat color_for_gra, depth_for_gra;

void on_trackbar_grasp();
static  void on_trackbar_grasp(int x, void*ustc);
vector<double> obtain_grasp_object_position( );




    string data_to_string();
    string data_to_biaoshi();





    int poly_order;
    VectorXd polyfit_2d_img( vector<Vector3d> &nav_waypoints_img);
    Vector3d pixel2Camera(int u, int v, double depth);








Q_SIGNALS:
    void frameUpdate();
    void navigation();
    void navigation_order();
    void auto_navigation();
    void auto_navigation_order();
    void grasp_object();
    void grasp_object_order();


private:
    rs2::pipeline pipe1; // realsense2 sdk, pipe for getting image packet from d435.
    rs2::config config; // realsense2 sdk, config for the camera.

    cv::Mat     img_color;
    cv::Mat     img_depth;
    cv::Mat     img_colorized_depth;
    rs2::frame  depth;

    CameraInfo camera_info;

    rs2::context                ctx;            // Create librealsense context for managing devices
    std::vector<rs2::pipeline>  pipelines;
    rs2::config cfg;

    cv::Mat     img_color1;
    cv::Mat     img_depth1;
    cv::Mat     img_colorized_depth1;
    rs2::frame  depth1;

    cv::Mat     img_color2;
    cv::Mat     img_depth2;
    cv::Mat     img_colorized_depth2;
    rs2::frame  depth2;



    const int g_nMaxalphavalue=1;
    int g_nAlphavalueslider;

    cv::Mat img_nav;
    bool is_estimate_ground_plane;
    cv::Mat img_estimate_plane;

    bool is_first_waypoint;
    bool is_realsense_launch;
    vector<Vector2d> nav_waypoints_cam_x_z;
    vector<Vector3d> nav_waypoints_img;
    vector<Vector3d> nav_waypoints_cam;
    VectorXd poly_k;


    vector<Vector3d> obj_garsppoints_img;
  vector<Vector3d> obj_grasppoints_cam;

    cv::Mat img_gra;
    bool is_first_grasppoint;
    Matrix4d cam_end;
vector< double > obj_grasppoints_end;



    bool is_first_autopoint;
    vector<Vector3d> auto_waypoints_img;
    vector<Vector3d> auto_waypoints_cam;





};



#endif // IMAGE_PROCESSOR_H
