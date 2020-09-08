#ifndef Show3D_H
#define Show3D_H

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

#include "example.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"

#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

#include "image_processor.h"

//// 3D ///////////////////////////////////////////
struct filter_slider_ui
{
    std::string name;
    std::string label;
    std::string description;
    bool is_int;
    float value;
    rs2::option_range range;

    bool render(const float3& location, bool enabled);
    static bool is_all_integers(const rs2::option_range& range);
};

class filter_options
{
public:
    filter_options(const std::string name, rs2::filter& filter);
    filter_options(filter_options&& other);
    std::string filter_name;                                   //Friendly name of the filter
    rs2::filter& filter;                                       //The filter in use
    std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
    std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
};

// Helper functions for rendering the UI
void render_ui(float w, float h, std::vector<filter_options>& filters);
// Helper function for getting data from the queues and updating the view
void update_data(rs2::frame_queue& data, rs2::frame& depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map);

//// 3D ///////////////////////////////////////////

class Show3D : public QThread
{
    Q_OBJECT
public:
    Show3D();
    virtual ~Show3D();
    void run();
    bool stop_run();
    bool init();
    bool stop=false;
    void setCamera_num(int n);

private:
    // camera
    int num;

    // Declare disparity transform from depth to disparity and vice versa
    const std::string disparity_filter_name = "Disparity";
    // Construct objects to manage view state
    glfw_state filtered_view_orientation{};
    // Declare pointcloud objects, for calculating pointclouds and texture mappings
    rs2::pointcloud filtered_pc;
    // Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    // Initialize a vector that holds filters and their options
    std::vector<filter_options> filters;
    // Declaring two concurrent queues that will be used to enqueue and dequeue frames from different threads
    rs2::frame_queue filtered_data;
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Maximum angle for the rotation of the pointcloud
    double max_angle;
    // We'll use rotation_velocity to rotate the pointcloud for a better view of the filters effects
    float rotation_velocity;
};
rs2::frame getDepth();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////






#endif
