/**
 * This code is used for launch realsense and get the aligned color and depth image.
 */

#include "image_processor.h"
#include <string>
#include <sstream>
#include <QImage>
#include <QMessageBox>

#include "socket_robot/utility.h"
#include "socket_robot/socket_robot.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include "Astar.h"
#include "Show3D .h"

#define CVUI_IMPLEMENTATION
#include "cvui.h"


using namespace std;
using namespace cv;
using namespace Qt;
using namespace Eigen;


rs2::frame global_depth;
rs2::frame global_depth_1;
rs2::frame global_depth_2;


ImageProcessor::ImageProcessor()
{
    is_first_waypoint = false;
    poly_order = 3;
    expShow = false;

}

ImageProcessor::~ImageProcessor()
{
    terminate();

    for (int i=0; i<pipelines.size(); i++)
    {
        pipelines.at(i).stop();
    }
    pipelines.clear();

    wait();
}

bool ImageProcessor::init()
{
    rs2::pipeline_profile profile;
    int i = 1;
        for (auto&& dev : ctx.query_devices())
        {
            rs2::pipeline  pipe(ctx);
            rs2::config cfg;
            cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

            cout << "serial_number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << endl;
            QString rgbd_id = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

            if(rgbd_id == RGBD_L_ID)
            {
                    rgbd_id_list[RGBD_L_ID] = i++;
                    cout << "RGBD_L_ID" << endl;
            }
            else if(rgbd_id == RGBD_C_ID)
            {
                    rgbd_id_list[RGBD_C_ID] = i++;
                    cout << "RGBD_C_ID" << endl;
            }
            else if(rgbd_id == RGBD_R_ID)
            {
                    rgbd_id_list[RGBD_R_ID] = i++;
                    cout << "RGBD_R_ID" << endl;
            }
            cout << "rgbd_id_list[RGBD_L_ID]: " << rgbd_id_list[RGBD_L_ID] << endl;
            cout << "rgbd_id_list[RGBD_C_ID]: " << rgbd_id_list[RGBD_C_ID] << endl;
            cout << "rgbd_id_list[RGBD_R_ID]: " << rgbd_id_list[RGBD_R_ID] << endl;

            try{
                profile = pipe.start(cfg);
                cout << "start success" << endl;
                pipelines.emplace_back(pipe);
            }
            catch(rs2::invalid_value_error error1) {
                    cout << "rs2::invalid_value_error start failure: " << error1.what() << endl;
            }
            catch(rs2::backend_error error2){
                 cout << "rs2::backend_error start failure: " << error2.what() << endl;
            }
            catch(rs2::error error3){
                 cout << "rs2::error: " << error3.what() << endl;
            }
        }



        cout << RGBD_L_ID << ": " << rgbd_id_list[RGBD_L_ID] << endl;
        cout << RGBD_R_ID << ": "<< rgbd_id_list[RGBD_R_ID] << endl;
        cout << RGBD_C_ID << ": "<< rgbd_id_list[RGBD_C_ID] << endl;

        if(pipelines.size() > 0)
        {
            is_realsense_launch = true;            

            float depth_scale = get_depth_scale(profile.get_device());


            cout << "depth_scale = " << depth_scale << endl;

            start();

            return true;
        }
        else
            return false;


}

bool ImageProcessor::stop()
{

    terminate();

    for (int i=0; i<pipelines.size(); i++)
    {
        pipelines.at(i).stop();
    }
    pipelines.clear();


    return true;
}

int ImageProcessor::realsense_numb()
{
    int m=pipelines.size();
    return m;
}
cv::Mat ImageProcessor:: toOCCImage(const rs2::frameset& framesets)
{
    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(framesets);
    rs2::video_frame color = aligned_frames.first(RS2_STREAM_COLOR);

    rs2::stream_profile pfile = color.get_profile();
    rs2_intrinsics intrinc = pfile.as<rs2::video_stream_profile>().get_intrinsics();
    camera_info.fx=intrinc.fx;
    camera_info.fy=intrinc.fy;
    camera_info.cx=intrinc.ppx;
    camera_info.cy=intrinc.ppy;
    camera_info.K = Matrix3d::Identity();
     auto &K = camera_info.K;
     K(0,0) = camera_info.fx;
     K(1,1) = camera_info.fy;
     K(0,2) = camera_info.cx;
     K(1,2) = camera_info.cy;




    cv::Mat img = color_RStoCV(color);

   return img;

}

rs2::depth_frame ImageProcessor:: toOCDImage(const rs2::frameset& framesets)
{

    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(framesets);

    rs2::depth_frame depth = aligned_frames.get_depth_frame();


    return depth;
}

rs2::depth_frame ImageProcessor:: toOCDImage_Show3D(const rs2::frameset& framesets)
{

    rs2::frame depth_Show3D = framesets.get_depth_frame(); //Take the depth frame from the frameset

    return depth_Show3D;

}

cv::Mat ImageProcessor:: depth_RStoCV(const rs2::depth_frame& depth)
{
    const int w_depth = depth.as<rs2::depth_frame>().get_width();
    const int h_depth = depth.as<rs2::depth_frame>().get_height();

//    cout << "w_depth h_depth" << w_depth << "*" << h_depth << endl;

    Mat image2(Size(w_depth,h_depth), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);
    img_depth = image2.clone();

    cv::Mat img = image2.clone();
    return img;
}

cv::Mat ImageProcessor:: color_RStoCV(const rs2::video_frame& color)
{
    const int w = color.as<rs2::video_frame>().get_width();
    const int h = color.as<rs2::video_frame>().get_height();

    Mat image1(Size(w,h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

    cv::Mat img = image1.clone();
    return img;
}

rs2::depth_frame ImageProcessor:: depth_filter(rs2::depth_frame depth)
{
    rs2::spatial_filter     spat_filter;
    rs2::temporal_filter    temp_filter;
    rs2::disparity_transform depth_to_disparity(true);

    depth = spat_filter.process(depth);
    depth = temp_filter.process(depth);
//    depth = depth_to_disparity.process(depth);

    return depth;
}


void ImageProcessor::run()
{
    /**
     * 1.将彩色图与深度图对齐，并且输出640*480的彩色图与深度图，且深度图的单位为mm，除以1000能够得到m
     * 2.获取了彩色相机的内参fx fy cx cy，畸变参数因为极其小，所以为0，或者可以理解为相机给你做好了去畸变。
     * 3.将sdk的图像格式转成了opencv格式。
     * 4.当采集到新的图像之后输出信号frameUpdate();
     */

    while(true)
    {

        std::vector<rs2::frameset> framesets;

              for (auto &&pipe : pipelines)
              {
                  rs2::frameset fs;

                  if (pipe.try_wait_for_frames(&fs))
                  {

                          framesets.emplace_back(fs);
                  }
              }

            if(pipelines.size() == 1)
            {
                if(rgbd_id_list[RGBD_L_ID] == 1)
                {
                    img_color = toOCCImage(framesets[0]);
                    depth = toOCDImage(framesets[0]);
                    img_depth = depth_RStoCV(depth);
                    global_depth = depth;
                }
                else if(rgbd_id_list[RGBD_R_ID] == 1)
                {
                    img_color1 = toOCCImage(framesets[0]);
                    depth1 = toOCDImage(framesets[0]);
                    img_depth1 = depth_RStoCV(depth1);
                    global_depth_1 = depth1;
                }
                else if(rgbd_id_list[RGBD_C_ID] == 1)
                {
                    img_color2 = toOCCImage(framesets[0]);
                    depth2 = toOCDImage(framesets[0]);
                    img_depth2 = depth_RStoCV(depth2);
                    global_depth_2 = depth2;
                }

            }
            else if(pipelines.size() == 2)
            {
                if(rgbd_id_list[RGBD_L_ID] == 1)
                {
                    img_color = toOCCImage(framesets[0]);
                    depth = toOCDImage(framesets[0]);
                    img_depth = depth_RStoCV(depth);
                    global_depth = depth;
                }
                else if(rgbd_id_list[RGBD_R_ID] == 1)
                {
                    img_color1 = toOCCImage(framesets[0]);
                    depth1 = toOCDImage(framesets[0]);
                    img_depth1 = depth_RStoCV(depth1);
                    global_depth_1 = depth1;
                }
                else if(rgbd_id_list[RGBD_C_ID] == 1)
                {
                    img_color2 = toOCCImage(framesets[0]);
                    depth2 = toOCDImage(framesets[0]);
                    img_depth2 = depth_RStoCV(depth2);
                    global_depth_2 = depth2;
                }


                if(rgbd_id_list[RGBD_L_ID] == 2)
                {
                    img_color = toOCCImage(framesets[1]);
                    depth = toOCDImage(framesets[1]);
                    img_depth = depth_RStoCV(depth);
                    global_depth = depth;
                }
                else if(rgbd_id_list[RGBD_R_ID] == 2)
                {
                    img_color1 = toOCCImage(framesets[1]);
                    depth1 = toOCDImage(framesets[1]);
                    img_depth1 = depth_RStoCV(depth1);
                    global_depth_1 = depth1;
                }
                else if(rgbd_id_list[RGBD_C_ID] == 2)
                {
                    img_color2 = toOCCImage(framesets[1]);
                    depth2 = toOCDImage(framesets[1]);
                    img_depth2 = depth_RStoCV(depth2);
                    global_depth_2 = depth2;
                }
            }
            else if(pipelines.size() == 3)
            {
                if(rgbd_id_list[RGBD_L_ID] == 1)
                {
                    img_color = toOCCImage(framesets[0]);
                    depth = toOCDImage(framesets[0]);
                    img_depth = depth_RStoCV(depth);
                    global_depth = depth;
                }
                else if(rgbd_id_list[RGBD_R_ID] == 1)
                {
                    img_color1 = toOCCImage(framesets[0]);
                    depth1 = toOCDImage(framesets[0]);
                    img_depth1 = depth_RStoCV(depth1);
                    global_depth_1 = depth1;
                }
                else if(rgbd_id_list[RGBD_C_ID] == 1)
                {
                    img_color2 = toOCCImage(framesets[0]);
                    depth2 = toOCDImage(framesets[0]);
                    img_depth2 = depth_RStoCV(depth2);
                    global_depth_2 = depth2;
                }


                if(rgbd_id_list[RGBD_L_ID] == 2)
                {
                    img_color = toOCCImage(framesets[1]);
                    depth = toOCDImage(framesets[1]);
                    img_depth = depth_RStoCV(depth);
                    global_depth = depth;
                }
                else if(rgbd_id_list[RGBD_R_ID] == 2)
                {
                    img_color1 = toOCCImage(framesets[1]);
                    depth1 = toOCDImage(framesets[1]);
                    img_depth1 = depth_RStoCV(depth1);
                    global_depth_1 = depth1;
                }
                else if(rgbd_id_list[RGBD_C_ID] == 2)
                {
                    img_color2 = toOCCImage(framesets[1]);
                    depth2 = toOCDImage(framesets[1]);
                    img_depth2 = depth_RStoCV(depth2);
                    global_depth_2 = depth2;
                }

                if(rgbd_id_list[RGBD_L_ID] == 3)
                {
                    img_color = toOCCImage(framesets[2]);
                    depth = toOCDImage(framesets[2]);
                    img_depth = depth_RStoCV(depth);
                    global_depth = depth;
                }
                else if(rgbd_id_list[RGBD_R_ID] == 3)
                {
                    img_color1 = toOCCImage(framesets[2]);
                    depth1 = toOCDImage(framesets[2]);
                    img_depth1 = depth_RStoCV(depth1);
                    global_depth_1 = depth1;
                }
                else if(rgbd_id_list[RGBD_C_ID] == 3)
                {
                    img_color2 = toOCCImage(framesets[2]);
                    depth2 = toOCDImage(framesets[2]);
                    img_depth2 = depth_RStoCV(depth2);
                    global_depth_2 = depth2;
                }

            }
            else
            {

            }


        Q_EMIT frameUpdate();

    }
}

QImage ImageProcessor::toQImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        cv::cvtColor(mat,mat,CV_BGR2RGB);
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        return QImage();
    }
}
/*
cv::Mat ImageProcessor::getImageColor()
{
    cv::Mat img = img_color.clone();
    return img;
}
*/
cv::Mat ImageProcessor::getImageColor()
{
    cv::Mat img = img_color.clone();
    Point beginpoint1=Point(105,480);
    Point beginpoint1_1=Point(100,480);
    Point beginpoint1_2=Point(110,480);
    Point endpoint1=Point(234,177);
    Point endpoint1_1=Point(229,177);
    Point endpoint1_2=Point(239,177);

    Point beginpoint2=Point(579,480);
    Point beginpoint2_1=Point(574,480);
    Point beginpoint2_2=Point(584,480);
     Point endpoint2=Point(424,180);
     Point endpoint2_1=Point(419,180);
     Point endpoint2_2=Point(429,180);


line(img,beginpoint1,endpoint1,Scalar(0,0,255),3,CV_AA);
line(img,beginpoint1_1,endpoint1_1,Scalar(255,0,0),3,CV_AA);
line(img,beginpoint1_2,endpoint1_2,Scalar(0,255,0),3,CV_AA);

line(img,beginpoint2,endpoint2,Scalar(0,0,255),3,CV_AA);
line(img,beginpoint2_1,endpoint2_1,Scalar(0,255,0),3,CV_AA);
line(img,beginpoint2_2,endpoint2_2,Scalar(255,0,0),3,CV_AA);


Point zuopoint1=Point(129,423);
Point youpoint1=Point(555,432);
line(img,zuopoint1,youpoint1,Scalar(255,0,0),1);

Point zuopoint2=Point(203,254);
Point youpoint2=Point(461,256);
line(img,zuopoint2,youpoint2,Scalar(0,0,255),1);
Point zuopoint3=Point(234,177);
Point youpoint3=Point(427,180);
line(img,zuopoint3,youpoint3,Scalar(0,255,0),1);




if(expShow){
    Point bz_zuopoint1=Point(272,480);
    Point bz_zuopoint2=Point(283,379);
    line(img,bz_zuopoint1,bz_zuopoint2,Scalar(0,255,0),3);

    Point bz_youpoint1=Point(459,480);
    Point bz_youpoint2=Point(450,387);
    line(img,bz_youpoint1,bz_youpoint2,Scalar(0,255,0),3);

    line(img,bz_zuopoint2,bz_youpoint2,Scalar(0,255,0),3);
}


    return img;
}

cv::Mat ImageProcessor::getImageDepth()
{
    cv::Mat img = img_depth.clone();
    return img;
}

rs2::frame  ImageProcessor::getDepth()
{
    rs2::frame  img = depth;
    return img;
}

cv::Mat ImageProcessor::getImageColorizedDepth()
{
    cv::Mat img = img_colorized_depth.clone();
    return img;
}

cv::Mat ImageProcessor::getImageColor1()
{
    cv::Mat img = img_color1.clone();
   return img;
}

cv::Mat ImageProcessor::getImageDepth1()
{
    cv::Mat img1 = img_depth1.clone();
   return img1;
}

cv::Mat ImageProcessor::getImageColorizedDepth1()
{
    cv::Mat img1 = img_colorized_depth1.clone();
    return img1;
}

cv::Mat ImageProcessor::getImageColor2()
{
    cv::Mat img2 = img_color2.clone();
   return img2;
}

cv::Mat ImageProcessor::getImageDepth2()
{
    cv::Mat img2 = img_depth2.clone();
   return img2;
}

cv::Mat ImageProcessor::getImageColorizedDepth2()
{
    cv::Mat img2 = img_colorized_depth2.clone();
    return img2;
}



float ImageProcessor::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}





//gujidimian


void ImageProcessor::on_mouse(int event,int x,int y,int flags,void *ustc)
{
    ImageProcessor* img_processor = reinterpret_cast<ImageProcessor*>(ustc);
    if(!(img_processor->is_set_plane_img))
    {
        cout << "Get the ground plane img.." << endl;
        img_processor->is_set_plane_img = true;
//        img_processor->color_for_plane = img_processor->getImageColor();
//        img_processor->depth_for_plane = img_processor->getImageDepth();
    }

    img_processor->on_mouse(event,x,y,flags,img_processor->color_for_plane, img_processor->depth_for_plane);
}

void ImageProcessor::on_mouse(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth)
{
        static Point pre_pt(-1,-1);
        static Point cur_pt(-1,-1);
        char temp[16];
        Mat org,img, tmp;
        cv::cvtColor(color,color,CV_BGR2RGB);
        org = color.clone();
        img = color.clone();

        if(color.empty())
        {
            cerr << "error" << endl;
        }
        if (event == CV_EVENT_LBUTTONDOWN)
        {
            img = org.clone();
            sprintf(temp,"(%d,%d)",x,y);
            pre_pt = Point(x,y);
            putText(img,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);
            circle(img,pre_pt,2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
            imshow("window_estimate_ground_plane",img);
        }
        else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))
        {
            img.copyTo(tmp);
            sprintf(temp,"(%d,%d)",x,y);
            cur_pt = Point(x,y);
            putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
            imshow("window_estimate_ground_plane",tmp);
        }
        else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
        {
            img.copyTo(tmp);
            sprintf(temp,"(%d,%d)",x,y);
            cur_pt = Point(x,y);
            putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
            rectangle(tmp,pre_pt,cur_pt,Scalar(255,0,0),1,8,0);
            imshow("window_estimate_ground_plane",tmp);
        }
        else if (event == CV_EVENT_LBUTTONUP)//左键放开
        {
            org.copyTo(img);
            sprintf(temp,"(%d,%d)",x,y);
            cur_pt = Point(x,y);
            putText(img,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
            circle(img,pre_pt,2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
            rectangle(img,pre_pt,cur_pt,Scalar(255,0,0),1,8,0);
            imshow("window_estimate_ground_plane",img);
            /// save the pts.
            cout << "pts:\n" << pre_pt.x << " " << pre_pt.y << " " << cur_pt.x << " " << cur_pt.y << endl;

            /// estimate the ground plane
            vector<Vector3d> pts_in_camera;
            for (int row = pre_pt.y; row < cur_pt.y; row+=2)
            {
                for (int col = pre_pt.x; col < cur_pt.x; col+=2)
                {
                    double val_depth = depth.at<ushort>(row, col)/1000.;
                    if(val_depth-1e-3 <=0. || val_depth > 20.0)
                        continue;
                    Vector3d pts = pixel2Camera(col, row, val_depth);
                    pts_in_camera.push_back(pts);
                }
            }
            pair<Vector3d,Vector3d> plane = best_plane_from_points(pts_in_camera);

            cout << "plane: " << plane.first << "  " << plane.second << endl;
            showGroundPlane(plane, img);


            imshow("window_estimate_ground_plane",img);

        }
}

void ImageProcessor::setGroundPlaneEstimateFlag(bool is_estimate)
{
    is_estimate_ground_plane = is_estimate;
}

void ImageProcessor::setRealsenseLaunchFlag(bool is_launch)
{
    is_realsense_launch = is_launch;
}

void ImageProcessor::estimateGroundPlane()
{
    if(cvGetWindowHandle("window_estimate_ground_plane"))
        destroyWindow("window_estimate_ground_plane");
    if(is_estimate_ground_plane && is_realsense_launch)
    {
        cout << "Do not move the robot!" << endl;
        namedWindow("window_estimate_ground_plane");
        setMouseCallback("window_estimate_ground_plane", on_mouse, this);
        img_estimate_plane = img_color.clone();
//        cvtColor(img_estimate_plane, img_estimate_plane, CV_BGR2RGB);
        color_for_plane = img_color.clone();
        depth_for_plane = img_depth.clone();
        imshow("window_estimate_ground_plane",img_estimate_plane);
        cout<<"nihao"<<endl;
        waitKey(0);
    }
    else
    {
        QMessageBox::information(NULL, "是否启动相机?", "请检查相机是否启动!", QMessageBox::Ok | QMessageBox::Cancel);
    }
}

pair<Vector3d, Vector3d> ImageProcessor::estimatePlane(const vector<Vector3d> &pts_uvd)
{
    vector<Vector3d> pts_cam;
//    assert(pts_cam.size() > 3);
    pts_cam.reserve(pts_uvd.size());
    for (int i = 0; i < pts_uvd.size(); ++i)
    {
        Vector3d pt_cam = pixel2Camera(pts_uvd[i](0), pts_uvd[i](1), pts_uvd[i](2));
        pts_cam.push_back(pt_cam);
    }
    assert(pts_cam.size() > 3);
    return best_plane_from_points(pts_cam);
}

void ImageProcessor::showGroundPlane(pair<Vector3d, Vector3d> &centroid_normal, cv::Mat &img)
{
    Vector3d &centor = centroid_normal.first;
    Vector3d &normal = centroid_normal.second;

    Matrix3d Rcp;
    Vector3d z_p = normal;
    Vector3d x_p(1.0, 0.0, 0.0);
    Vector3d y_p = z_p.cross(x_p);
    y_p.normalize();//单位化
    Rcp.block<3,1>(0,0) = x_p;//将x_p的值赋给Rcp中，(0,0)为起点，大小为<3,1>的区域
    Rcp.block<3,1>(0,1) = y_p;
    Rcp.block<3,1>(0,2) = z_p;

    Vector3d tcp = centor;

    cout << "Rcp =\n" << Rcp << endl;
    cout << "Tcp =\n" << tcp << endl;


    vector<Vector2d> pts_uv_north = grid.project_pts_to_image(grid.grid_north, Rcp, tcp, camera_info.K);//grid.grid_north的值为（-1.5，-1.5,0），（-1.2，-1.5,0）...（1.5,1.5,0）
    vector<Vector2d> pts_uv_south = grid.project_pts_to_image(grid.grid_south, Rcp, tcp, camera_info.K);
    vector<Vector2d> pts_uv_west = grid.project_pts_to_image(grid.grid_west, Rcp, tcp, camera_info.K);
    vector<Vector2d> pts_uv_east = grid.project_pts_to_image(grid.grid_east, Rcp, tcp, camera_info.K);

    /// line
    for (int i = 0; i < pts_uv_north.size(); ++i)
    {
        cout << "pts_uv_north"<<pts_uv_north[i] << endl;
        line(img, Point(pts_uv_north[i](0), pts_uv_north[i](1)), Point(pts_uv_south[i](0), pts_uv_south[i](1)), Scalar(0,255,0));//以north为起点，south为终点画一条直线
        line(img, Point(pts_uv_west[i](0), pts_uv_west[i](1)), Point(pts_uv_east[i](0), pts_uv_east[i](1)), Scalar(0,255,0));//以west为起点，east为终点画一条直线
    }

}

std::pair<Vector3d, Vector3d> ImageProcessor::best_plane_from_points(const std::vector<Vector3d> & c)//估计最优平面
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix< Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

    // calculate centroid
    Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3d plane_normal = svd.matrixU().rightCols<1>();
    if(plane_normal(1) < 0)
        plane_normal = -plane_normal;
    plane_normal.normalize();
    return std::make_pair(centroid, plane_normal);
}





//navigation



void ImageProcessor::on_mouse_way_points(int event, int x, int y, int flags, void *ustc)
{
    ImageProcessor* img_processor = reinterpret_cast<ImageProcessor*>(ustc);

    img_processor->on_mouse_way_points(event, x, y, flags, img_processor->color_for_nav, img_processor->depth_for_nav);
}

void ImageProcessor::on_mouse_way_points(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth)
{
    static vector<Point> waypoints;
    static Point waypoint;
    char temp[16];
    Mat org,img, tmp;
    org = color.clone();
    img = color.clone();
    if(color.empty())
    {
        cerr << "error" << endl;
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {

        if(true)
        {
            sprintf(temp,"(%d,%d)",x,y);
            waypoint = Point(x,y);

            putText(img,temp,waypoint,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);

            if(is_first_waypoint)
            {

                is_first_waypoint = false;
                circle(img,waypoint,5,Scalar(0,255,0),CV_FILLED,CV_AA,0);
                Eigen::Vector3d pt_uvd(waypoint.x, waypoint.y, depth.at<ushort>(waypoint.y, waypoint.x)/1000.);
                nav_waypoints_img.emplace_back(pt_uvd);
            }
            else
            {
                circle(img,Point(nav_waypoints_img[0](0),nav_waypoints_img[0](1)),5,Scalar(0,255,0),CV_FILLED,CV_AA,0);
                for (int i = 1; i < nav_waypoints_img.size(); ++i)
                {
                    circle(img,Point(nav_waypoints_img[i](0),nav_waypoints_img[i](1)),2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
                }
                circle(img,waypoint,2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
                Eigen::Vector3d pt_uvd(waypoint.x, waypoint.y, depth.at<ushort>(waypoint.y, waypoint.x)/1000.);
                nav_waypoints_img.emplace_back(pt_uvd);
            }
        }

        imshow("Navigation Setup.",img);
    }
    else if(event == CV_EVENT_RBUTTONDOWN)
    {
        Eigen::Vector3d start_uvd(320, 480, depth.at<ushort>(320, 480)/1000.);
        nav_waypoints_img.emplace_back(start_uvd);

        poly_order=3;
        if(nav_waypoints_img.size() > poly_order+1)
        {
            poly_k = polyfit_2d_img(nav_waypoints_img);
            /// Debug Show
            for (int y = (int)nav_waypoints_img[0].y(); y < img.rows; ++y)
            {
                VectorXd Y(poly_order+1);
                for (int i = 0; i < poly_order+1; ++i)
                {
                    Y(i) = pow(y, i);
                }
                int x = poly_k.transpose() * Y;
                circle(img,Point(x, y),6,Scalar(0,0,255),CV_FILLED,CV_AA,0);
            }

            for (int i = 1; i < nav_waypoints_img.size(); ++i)
            {
                circle(img,Point(nav_waypoints_img[i](0),nav_waypoints_img[i](1)),2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
            }
            pair<Vector3d, Vector3d> centroid_normal = estimatePlane(nav_waypoints_img);


    for (int i = 0; i < nav_waypoints_img.size(); ++i)
    {
         Vector2d pt_cam_x_z = pixel2Camera_x_z(nav_waypoints_img[i](0), nav_waypoints_img[i](2));
         nav_waypoints_cam_x_z.push_back(pt_cam_x_z);

         Vector3d pt_cam = pixel2Camera(nav_waypoints_img[i](0), nav_waypoints_img[i](1),  nav_waypoints_img[i](2));
         nav_waypoints_cam.push_back(pt_cam);



    }


            cv::cvtColor(img,img,CV_BGR2RGB);
            imshow("Navigation Setup.",img);
        }
        else
        {
            QMessageBox::information(NULL, "导航点选择", "请选择一共超过4个点!", QMessageBox::Ok | QMessageBox::Cancel);
        }


        for (int i = 0; i < nav_waypoints_cam.size(); ++i)
        {

            cout<<"图像轨迹点坐标："<<"("<<nav_waypoints_img[i](0)<<","<<nav_waypoints_img[i](1)<<","<<nav_waypoints_img[i](2)<<")"<<endl;

        }
   //         cout << "intrinc: \n";
   //         cout << camera_info.fx << " " << camera_info.fy << " " <<  camera_info.cx << " " << camera_info.cy << endl;

      Q_EMIT navigation();

    }

    else if(event == CV_EVENT_LBUTTONDBLCLK)
    {
       cout<<"123";
     Q_EMIT     navigation_order();

    }



}
vector<Vector2d> ImageProcessor::obtain_navigation_waypoints( )
{
     vector<Vector2d> nav_waypoints_cam_x_z;
    for (int i = nav_waypoints_cam.size()-2; i >= 0; --i)
    {
         Vector2d pt_cam_x_z;
         pt_cam_x_z(0)=nav_waypoints_cam[i](0);
         pt_cam_x_z(1)=nav_waypoints_cam[i](2);
         nav_waypoints_cam_x_z.push_back(pt_cam_x_z);

    }
    cout<<"发送的路径点数："<<nav_waypoints_cam_x_z.size()<<endl;
    for (int i = 0; i < nav_waypoints_cam_x_z.size(); ++i)
    {
        cout<<"发送的路径点:"<<"("<<nav_waypoints_cam_x_z[i](0)<<","<<nav_waypoints_cam_x_z[i](1)<<")"<<endl;

    }
    return nav_waypoints_cam_x_z;
}





void ImageProcessor::setWayPoints()
{
    /// check the estimate is OK?
    is_first_waypoint = true;
    if(cvGetWindowHandle("Navigation Setup."))
        destroyWindow("Navigation Setup.");
    if((!nav_waypoints_img.empty()) || (!nav_waypoints_cam.empty()))
    {
        nav_waypoints_img.clear();
        nav_waypoints_cam.clear();
    }

    if(is_realsense_launch)
    {
        cout << "Do not move the robot!" << endl;/***************************************************************/
        namedWindow("Navigation Setup.");
        setMouseCallback("Navigation Setup.", on_mouse_way_points, this);
        img_nav = img_color.clone();
        cvtColor(img_nav, img_nav, CV_BGR2RGB);
        color_for_nav = img_color.clone();
        depth_for_nav = img_depth.clone();


         g_nAlphavalueslider=0;
         char Trackbarname[50];
         sprintf(Trackbarname,"命令排爆车移动%d",g_nMaxalphavalue);
         createTrackbar(Trackbarname,"Navigation Setup.",&g_nAlphavalueslider,g_nMaxalphavalue, on_trackbar,this);


        imshow("Navigation Setup.",img_nav);
        waitKey(0);
    }
    else
    {
        QMessageBox::information(NULL, "是否启动相机?", "请检查相机是否启动!", QMessageBox::Ok | QMessageBox::Cancel);
    }

}

void ImageProcessor::on_trackbar(int x,void *ustc)
{

    ImageProcessor* img_processor= reinterpret_cast<ImageProcessor*>(ustc);
if (x==1)
{
    img_processor->on_trackbar( );
}

}

void ImageProcessor::on_trackbar()
{

    Q_EMIT navigation_order();

}




VectorXd ImageProcessor::polyfit_2d_img(vector<Vector3d> &nav_waypoints_img)
{

    int size = nav_waypoints_img.size();
    assert(size > (poly_order+1));

    int y_num = poly_order + 1;


    MatrixXd U(size, y_num);
    MatrixXd X(size, 1);
    for (int i = 0; i < U.rows(); ++i)
    {
        for (int j = 0; j < U.cols(); ++j)
        {
            double y = static_cast<double>(nav_waypoints_img[i](1));
            U(i,j) = pow(y, j);
        }
    }

    for (int k = 0; k < X.rows(); ++k)
    {
        X(k,0) = static_cast<double>(nav_waypoints_img[k](0));
    }

    VectorXd K = (U.transpose() * U).inverse() * U.transpose() * X;

    cout << K << endl;/*******************************************************************************************/
    return  K;
}


Vector3d ImageProcessor::pixel2Camera(int u, int v, double depth)
{
    double x = (u - camera_info.cx)*depth / camera_info.fx;
    double y = (v - camera_info.cy)*depth / camera_info.fy;
    Vector3d p(x, y, depth);
    return p;
}
Vector2d ImageProcessor::pixel2Camera_x_z(int u, double depth)
{
    double x = (u - camera_info.cx)*depth / camera_info.fx;

    Vector2d p(x, depth);
    return p;
}











//auto-navigation

void ImageProcessor::on_auto_way_points(int event, int x, int y, int flags, void *ustc)
{
    ImageProcessor* img_processor = reinterpret_cast<ImageProcessor*>(ustc);

    img_processor->on_auto_way_points(event, x, y, flags, img_processor->color_for_auto, img_processor->depth_for_auto);
}
void ImageProcessor::on_auto_way_points(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth)
{
    static vector<Point> waypoints;
    static Point waypoint;
    char temp[16];
    Mat org,img, tpyt,tp;
     int T=2080,N=480;//T=280;
    double a=0.05;
    double b=0.03;
    double c=0.05;
    double d=0.05;
    double e=0.05;
    vector<double> x_data, y_data;
    vector<double> tpy;
    static Point endpoint;


    org = color.clone();
    img = color.clone();
    tpyt=depth.clone();
    img_auto=Mat::zeros(480,640,CV_8UC1);
    if(color.empty())
    {
        cerr << "error" << endl;
    }

    //medianBlur(tpyt,tp,3);
    GaussianBlur(tpyt,tp,Size(9,9),5,5);
    if (event == CV_EVENT_LBUTTONDOWN)
    {


        sprintf(temp,"(%d,%d)",x,y);
        endpoint = Point(x,y);
        putText(img,temp,endpoint,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);

        if(true)
        {

            for(int i=0;i<480;i++)
           {
                double MaxValue,MinValue;
                x_data.push_back(i+1);

               Mat max_row=tp.row(i).clone();
               minMaxLoc(max_row,&MinValue,&MaxValue);
                y_data.push_back(MaxValue);

            }


                ceres::Problem problem;
                for ( int i=0; i<N; i++ )
                {
                    problem.AddResidualBlock (

                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 1,1,1,1,1> (
                            new CURVE_FITTING_COST ( x_data[i], y_data[i] )
                        ),
                        nullptr,
                        &a,&b,&c,&d,&e
                    );
                }

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = true;

                ceres::Solver::Summary summary;

                ceres::Solve ( options, &problem, &summary );


                cout<<summary.BriefReport() <<endl;
                cout<<"estimated a,b,c,d = ";
cout << "Final   a: " << a << " b: " << b <<  " c: " << c<< " d: " << d<<"\n";
                cout<<endl;

                for(int i=0;i<480;i++)
               {
                    tpy.push_back(a*pow(x_data[i],4) + b*pow(x_data[i],3)+c*pow(x_data[i],2)+d*pow(x_data[i],1)+e);
                  //  tpy.push_back(a*exp(b*x_data[i]) + c*exp(d*x_data[i]));


                }



                for(int i=0;i<640;i++)
               {
                    for(int j=0;j<480;j++)
                    {
                        if(abs(tpyt.at<ushort>(j,i)-tpy[j])>T)
                        {
                            img.at<Vec3b>(j,i)[0]=255;
                            img_auto.at<ushort>(j,i)=1;

                        }

 //                       cout<<"tp："<<abs(tp.at<ushort>(j,i)-tpy[j])<<endl;

                    }

                }

        }

        imshow("Auto_Navigation Setup.",img);
    }
    else if(event == CV_EVENT_RBUTTONDOWN)
    {


        vector<vector<int>> vec;

        for(int i=0;i<480;i++)
       {
           vec.push_back(img_auto.row(i).clone());
        }
        cout<<"程序运行到此"<<vec.size()<<endl;
        Astar astar;
        astar.InitAstar(vec);

        Pointa start(479,319);
        Pointa end(endpoint.y,endpoint.x);
     //   Pointa end(300,300);

        list<Pointa *> path=astar.GetPath(start,end,false);

           for(auto &p:path)
           {

               cout<<'('<<p->x<<','<<p->y<<')'<<endl;
        waypoint = Point(p->y,p->x);
        cout<<"waypoint:"<<waypoint.x<<','<<waypoint.y<<endl;
        circle(img,waypoint,5,Scalar(0,255,0),CV_FILLED,CV_AA,0);
        Eigen::Vector3d pt_uvd(waypoint.x, waypoint.y, depth.at<ushort>(waypoint.y, waypoint.x)/1000.);
        auto_waypoints_img.emplace_back(pt_uvd);

           }

          cout<<"真实的路径点数："<<auto_waypoints_img.size()<<endl;
          int  m=auto_waypoints_img.size();
          int n1=m/11;
          int n2=m/2;

/*                  for (int i = n1; i < m; )
                  {
                       Vector3d pt_cam = pixel2Camera(auto_waypoints_img[i](0), auto_waypoints_img[i](1), auto_waypoints_img[i](2));
                       auto_waypoints_cam.push_back(pt_cam);
                       i=i+n1;

                  }

*/

                  for (int i = n1; i < n2; )
          {
               Vector3d pt_cam = pixel2Camera(auto_waypoints_img[i](0), auto_waypoints_img[i](1), auto_waypoints_img[i](2));
               auto_waypoints_cam.push_back(pt_cam);
               i=i+n1*1.6;

          }
          for (int j = n2; j < m; )
          {
               Vector3d pt_cam = pixel2Camera(auto_waypoints_img[j](0), auto_waypoints_img[j](1), auto_waypoints_img[j](2));
               auto_waypoints_cam.push_back(pt_cam);
               j=j+n1/1.6;

          }





          /*         for (int i = 0; i < auto_waypoints_img.size(); ++i)
          {
               Vector3d pt_cam = pixel2Camera(auto_waypoints_img[i](0), auto_waypoints_img[i](1), auto_waypoints_img[i](2));
               auto_waypoints_cam.push_back(pt_cam);


          }*/
           cv::cvtColor(img,img,CV_BGR2RGB);
           imshow("Auto_Navigation Setup.",img);
           Q_EMIT auto_navigation();


    }

}

vector<Vector2d> ImageProcessor::obtain_auto_navigation_waypoints( )
{
     vector<Vector2d> auto_waypoints_cam_x_z;
    for (int i = 0; i < auto_waypoints_cam.size(); )
    {
         Vector2d pt_cam_x_z;
         pt_cam_x_z(0)=auto_waypoints_cam[i](0);
         pt_cam_x_z(1)=auto_waypoints_cam[i](2);
         auto_waypoints_cam_x_z.push_back(pt_cam_x_z);
         i=i+1;

    }
    cout<<"发送的路径点数："<<auto_waypoints_cam_x_z.size()<<endl;


    return auto_waypoints_cam_x_z;
}

void ImageProcessor::on_trackbar_auto(int x,void *ustc)
{
    ImageProcessor* img_processor= reinterpret_cast<ImageProcessor*>(ustc);
if (x==1)
{
    img_processor->on_trackbar_auto();
}
}
void ImageProcessor::on_trackbar_auto()
{

   Q_EMIT     auto_navigation_order();
}



void ImageProcessor::autoWayPoints()
{
    /// check the estimate is OK?
    is_first_autopoint = true;
    if(cvGetWindowHandle("Auto_Navigation Setup."))
        destroyWindow("Auto_Navigation Setup.");
    if((!auto_waypoints_img.empty()) || (!auto_waypoints_cam.empty()))
    {
        auto_waypoints_img.clear();
        auto_waypoints_cam.clear();

    }

    if(is_realsense_launch)
    {
        cout << "Do not move the robot!" << endl;/***************************************************************/
        namedWindow("Auto_Navigation Setup.");
        setMouseCallback("Auto_Navigation Setup.", on_auto_way_points, this);
        img_nav = img_color.clone();
        cvtColor(img_nav, img_nav, CV_BGR2RGB);
        color_for_auto = img_color.clone();
        depth_for_auto = img_depth.clone();
        imwrite("color_for_auto.png",img_nav);

        imwrite("depth_for_auto.png",depth_for_auto);
        g_nAlphavalueslider=0;
        char Trackbarname[50];
        sprintf(Trackbarname,"命令排爆车移动%d",g_nMaxalphavalue);
        createTrackbar(Trackbarname,"Auto_Navigation Setup.",&g_nAlphavalueslider,g_nMaxalphavalue, on_trackbar_auto,this);


        imshow("Auto_Navigation Setup.",img_nav);
        waitKey(0);
    }
    else
    {
        QMessageBox::information(NULL, "是否启动相机?", "请检查相机是否启动!", QMessageBox::Ok | QMessageBox::Cancel);
    }

}

//zhuaquwuti
void ImageProcessor::setgraspPoints()
{
    // check the estimate is OK?
    is_first_grasppoint = true;
    if(cvGetWindowHandle("Grasp Setup."))
        destroyWindow("Grasp Setup.");
    if((!obj_garsppoints_img.empty()) || (!obj_grasppoints_cam.empty()))
    {
        obj_garsppoints_img.clear();
        obj_grasppoints_cam.clear();
        obj_grasppoints_end.clear();
    }

    if(is_realsense_launch)
    {
        cout << "Do not move the robot!" << endl;/***************************************************************/
        namedWindow("Grasp Setup.");
        setMouseCallback("Grasp Setup.", on_mouse_grasp_points, this);
        img_gra = img_color2.clone();
        cvtColor(img_gra, img_gra, CV_BGR2RGB);
        color_for_gra = img_color2.clone();
        depth_for_gra = img_depth2.clone();


         g_nAlphavalueslider=0;
         char Trackbarname[50];
         sprintf(Trackbarname,"命令机械臂移动%d",g_nMaxalphavalue);
         createTrackbar(Trackbarname,"Grasp Setup.",&g_nAlphavalueslider,g_nMaxalphavalue, on_trackbar_grasp,this );
         //on_trackbar(g_nAlphavalueslider,0);


        imshow("Grasp Setup.",img_gra);

vector<int>::size_type ix=0;
while(ix!=obj_garsppoints_img.size()){
cout<<obj_garsppoints_img[ix++]<<" ";
}
cout<<endl;

        waitKey(0);
    }
    else
    {
        QMessageBox::information(NULL, "是否启动相机?", "请检查相机是否启动!", QMessageBox::Ok | QMessageBox::Cancel);
    }

}

vector<double> ImageProcessor::obtain_grasp_object_position( )
{

    cout<<"物体位姿数据数量："<<obj_grasppoints_end.size()<<endl;

    return obj_grasppoints_end;
}


void ImageProcessor::on_trackbar_grasp(int x,void *ustc)
{
    ImageProcessor* img_processor= reinterpret_cast<ImageProcessor*>(ustc);
if (x==1)
{
    img_processor->on_trackbar_grasp();
}
}
void ImageProcessor::on_trackbar_grasp()
{

   Q_EMIT     grasp_object_order();
}



void ImageProcessor::on_mouse_grasp_points(int event, int x, int y, int flags, void *ustc)
{
    ImageProcessor* img_processor = reinterpret_cast<ImageProcessor*>(ustc);

    img_processor->on_mouse_grasp_points(event, x, y, flags, img_processor->color_for_gra, img_processor->depth_for_gra);
}

void ImageProcessor::on_mouse_grasp_points(int event, int x, int y, int flags, cv::Mat &color, cv::Mat &depth)
{
    static vector<Point> waypoints;
    static Point waypoint;
    char temp[16];
    Mat org,img, tmp;
    org = color.clone();
    img = color.clone();
    Matrix4d object_coordinate_cam;

    if(color.empty())
    {
        cerr << "error" << endl;
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {

        if(true)
        {
            sprintf(temp,"(%d,%d)",x,y);
            waypoint = Point(x,y);

            putText(img,temp,waypoint,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);


            if(is_first_grasppoint)
            {


                is_first_grasppoint = false;
                circle(img,waypoint,5,Scalar(0,255,0),CV_FILLED,CV_AA,0);

                Eigen::Vector3d pt_uvd(waypoint.x, waypoint.y, depth.at<ushort>(waypoint.y, waypoint.x)/1000.);
                obj_garsppoints_img.emplace_back(pt_uvd);
                waypoints.emplace_back(waypoint);
            }
            else
            {

                circle(img,Point(obj_garsppoints_img[0](0),obj_garsppoints_img[0](1)),5,Scalar(0,255,0),CV_FILLED,CV_AA,0);
                for (int i = 1; i < obj_garsppoints_img.size(); ++i)
                {
                    circle(img,Point(obj_garsppoints_img[i](0),obj_garsppoints_img[i](1)),2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
                }
                circle(img,waypoint,2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
                Eigen::Vector3d pt_uvd(waypoint.x, waypoint.y, depth.at<ushort>(waypoint.y, waypoint.x)/1000.);
                obj_garsppoints_img.emplace_back(pt_uvd);
                waypoints.emplace_back(waypoint);
            }
        }

        imshow("Grasp Setup.",img);

    }

    else if(event == CV_EVENT_RBUTTONDOWN)
    {


        for (int i = 0; i < obj_garsppoints_img.size(); ++i)
        {
             Vector3d pt_cam = pixel2Camera(obj_garsppoints_img[i](0), obj_garsppoints_img[i](1), obj_garsppoints_img[i](2));
             obj_grasppoints_cam.push_back(pt_cam);


        }



                object_coordinate_cam<<0,0,0,0,
                                       0,0,0,0,
                                       0,0,0,0,
                                       0,0,0,1;

                Vector3d aa,bb,cc,dd,ee;
                Vector3d aa_norm,bb_norm;

                aa=obj_grasppoints_cam[0]-obj_grasppoints_cam[1];
                bb=obj_grasppoints_cam[2]-obj_grasppoints_cam[1];
                cc=obj_grasppoints_cam[2]-obj_grasppoints_cam[0];
                dd(0)=aa.norm();
                dd(1)=bb.norm();
                aa_norm=aa/dd(0);
                bb_norm=bb/dd(1);

                if(dd(0)>dd(1))
                {

                    ee=bb_norm.cross(aa_norm);
                    object_coordinate_cam(0,0)=bb_norm(0);
                    object_coordinate_cam(1,0)=bb_norm(1);
                    object_coordinate_cam(2,0)=bb_norm(2);
                    object_coordinate_cam(0,1)=aa_norm(0);
                    object_coordinate_cam(1,1)=aa_norm(1);
                    object_coordinate_cam(2,1)=aa_norm(2);
                    object_coordinate_cam(0,2)=ee(0);
                    object_coordinate_cam(1,2)=ee(1);
                    object_coordinate_cam(2,2)=ee(2);


                }
                else
                {
                    ee=aa_norm.cross(bb_norm);
                    object_coordinate_cam(0,0)=aa_norm(0);
                    object_coordinate_cam(1,0)=aa_norm(1);
                    object_coordinate_cam(2,0)=aa_norm(2);
                    object_coordinate_cam(0,1)=bb_norm(0);
                    object_coordinate_cam(1,1)=bb_norm(1);
                    object_coordinate_cam(2,1)=bb_norm(2);
                    object_coordinate_cam(0,2)=ee(0);
                    object_coordinate_cam(1,2)=ee(1);
                    object_coordinate_cam(2,2)=ee(2);

                }
                object_coordinate_cam(0,3)=obj_grasppoints_cam[1](0);
                object_coordinate_cam(1,3)=obj_grasppoints_cam[1](1);
                object_coordinate_cam(2,3)=obj_grasppoints_cam[1](2);

                Vector4d o_zxd(0,0,0,1);
                o_zxd(0)=obj_grasppoints_cam[3](0);
                o_zxd(1)=obj_grasppoints_cam[3](1);
                o_zxd(2)=obj_grasppoints_cam[3](2);


                cam_end<<0.998848798735267,   0.045055515964462,   0.016464438839236,  -0.03,
                         -0.045221845592191,   0.998928174826205,   0.009873511007887,  -0.055,
                         -0.016001935706374,  -0.010606696920477,   0.999815701033989,   -0.055,
                                         0,                   0,                   0,   1.000000000000000;


                Vector4d object_center_end;
                Matrix4d object_coordinate_end;
                object_center_end=cam_end*o_zxd;

                object_coordinate_end=cam_end*object_coordinate_cam;

                cout<<"物体表面中心点在相机坐标系下的坐标：" <<o_zxd<<endl;
                cout<<"物体在机械臂末端的坐标系下的位姿：" <<object_coordinate_end<<endl;
                cout<<"物体表面中心点在机械臂末端坐标系下的坐标：" <<object_center_end<<endl;

                obj_grasppoints_end.push_back(object_coordinate_end(0,0));
                obj_grasppoints_end.push_back(object_coordinate_end(0,1));
                obj_grasppoints_end.push_back(object_coordinate_end(0,2));
                obj_grasppoints_end.push_back(object_coordinate_end(1,0));
                obj_grasppoints_end.push_back(object_coordinate_end(1,1));
                obj_grasppoints_end.push_back(object_coordinate_end(1,2));
                obj_grasppoints_end.push_back(object_coordinate_end(2,0));
                obj_grasppoints_end.push_back(object_coordinate_end(2,1));
                obj_grasppoints_end.push_back(object_coordinate_end(2,2));
                //obj_grasppoints_end.push_back(object_coordinate_end(0,3));
                //obj_grasppoints_end.push_back(object_coordinate_end(1,3));
                //obj_grasppoints_end.push_back(object_coordinate_end(2,3));

                  obj_grasppoints_end.push_back(object_center_end(0));
                  obj_grasppoints_end.push_back(object_center_end(1));
                  obj_grasppoints_end.push_back(object_center_end(2));

                  int juxing_forth_x;
                  int juxing_forth_y;
                  juxing_forth_x=obj_garsppoints_img[0](0)+obj_garsppoints_img[2](0)-obj_garsppoints_img[1](0);
                  juxing_forth_y=obj_garsppoints_img[0](1)+obj_garsppoints_img[2](1)-obj_garsppoints_img[1](1);

                  line(img,Point(obj_garsppoints_img[0](0),obj_garsppoints_img[0](1)),Point(obj_garsppoints_img[1](0),obj_garsppoints_img[1](1)),Scalar(0,255,0),3,CV_AA);
                  line(img,Point(obj_garsppoints_img[1](0),obj_garsppoints_img[1](1)),Point(obj_garsppoints_img[2](0),obj_garsppoints_img[2](1)),Scalar(0,255,0),3,CV_AA);
                  line(img,Point(obj_garsppoints_img[2](0),obj_garsppoints_img[2](1)),Point(juxing_forth_x,juxing_forth_y),Scalar(0,255,0),3,CV_AA);
                  line(img,Point(obj_garsppoints_img[0](0),obj_garsppoints_img[0](1)),Point(juxing_forth_x,juxing_forth_y),Scalar(0,255,0),3,CV_AA);


         //       rectangle(img,Point(obj_garsppoints_img[0](0),obj_garsppoints_img[0](1)),Point(obj_garsppoints_img[2](0),obj_garsppoints_img[2](1)),Scalar(0,255,0),2,8,0);


                cv::cvtColor(img,img,CV_BGR2RGB);
                imshow("Grasp Setup.",img);

                Q_EMIT grasp_object();






    }


}







