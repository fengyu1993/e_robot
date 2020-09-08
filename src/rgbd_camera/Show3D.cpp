#include "Show3D .h"
#include <string>
#include <sstream>
#include <QImage>
#include <QMessageBox>

#include "socket_robot/utility.h"
#include "socket_robot/socket_robot.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include "Astar.h"


using namespace std;
using namespace cv;
using namespace Qt;

extern rs2::frame global_depth;
extern rs2::frame global_depth_1;
extern rs2::frame global_depth_2;

Show3D::Show3D()
{
//    num = n;
    // Maximum angle for the rotation of the pointcloud
    max_angle = 0.0;
    // We'll use rotation_velocity to rotate the pointcloud for a better view of the filters effects
    rotation_velocity = 0.0f;
    filtered_view_orientation.yaw = -0.0;

}

Show3D::~Show3D()
{
    terminate();
    wait();
}

bool Show3D::init()
{
    start();
    stop = false;
    cout << "Show3D Init Start" << endl;
    return true;
}

void Show3D::run()
{ 

    // Create a simple OpenGL window for rendering:
    example::window app3d(640, 480, "3D");
    ImGui_ImplGlfw_Init(app3d, false);

    // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    // The following order of emplacement will dictate the orders in which filters are applied
    filters.emplace_back("Decimate", dec_filter);
    //filters.emplace_back("Threshold", thr_filter);
    filters.emplace_back(disparity_filter_name, depth_to_disparity);
    //filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);


    // Atomic boolean to allow thread safe way to stop the thread
    std::atomic_bool stopped(false);

    // Create a thread for getting frames from the device and process them
    // to prevent UI thread from blocking due to long computations.
    std::thread processing_thread([&]() {
        while (!stopped) //While application is running
        {
            rs2::frame depth;

            switch (num) {
            case 1:
                depth = global_depth;
                break;
            case 2:
                depth = global_depth_1;
                break;
            case 3:
                depth = global_depth_2;
                break;
            default:
                break;
            }

//            rs2::frame depth = global_depth;

            if (!depth) // Should not happen but if the pipeline is configured differently
                return;       //  it might not provide depth and we don't want to crash

            rs2::frame filtered = depth; // Does not copy the frame, only adds a reference

            /* Apply filters.
            The implemented flow of the filters pipeline is in the following order:
            1. apply decimation filter
            2. apply threshold filter
            3. transform the scene into disparity domain
            4. apply spatial filter
            5. apply temporal filter
            6. revert the results back (if step Disparity filter was applied
            to depth domain (each post processing block is optional and can be applied independantly).
            */
            bool revert_disparity = false;
            for (auto&& filter : filters)
            {
                if (filter.is_enabled)
                {
                    filtered = filter.filter.process(filtered);
                    if (filter.filter_name == disparity_filter_name)
                    {
                        revert_disparity = true;
                    }
                }
            }
            if (revert_disparity)
            {
                filtered = disparity_to_depth.process(filtered);
            }

            // Push filtered & original data to their respective queues
            // Note, pushing to two different queues might cause the application to display
            //  original and filtered pointclouds from different depth frames
            //  To make sure they are synchronized you need to push them together or add some
            //  synchronization mechanisms
            filtered_data.enqueue(filtered);
       }
    });

    rs2::frame colored_depth;
    rs2::frame colored_filtered;
    rs2::points original_points;
    rs2::points filtered_points;

    // Save the time of last frame's arrival
    auto last_time = std::chrono::high_resolution_clock::now();

    while (app3d)
    {
        float w = static_cast<float>(app3d.width());
        float h = static_cast<float>(app3d.height());

        update_data(filtered_data, colored_filtered, filtered_points, filtered_pc, filtered_view_orientation, color_map);

        if (colored_filtered && filtered_points)
        {
            glViewport(0, 0, int(w) , int(h));
            draw_pointcloud(int(w) , int(h) , filtered_view_orientation, filtered_points);
        }
    }

    // Signal the processing thread to stop, and join
    // (Not the safest way to join a thread, please wrap your threads in some RAII manner)
    stopped = true;
    processing_thread.join();
}

bool Show3D::stop_run()
{
    terminate();
}

void Show3D::setCamera_num(int n)
{
    num = n;
}

bool filter_slider_ui::render(const float3& location, bool enabled)
{
    bool value_changed = false;
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 20 / 255.f, 150 / 255.f, 70 / 255.f, 1 });
    ImGui::GetStyle().GrabRounding = 12;
    if (!enabled)
    {
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 0,0,0,0 });
        ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 0,0,0,0 });
        ImGui::PushStyleColor(ImGuiCol_Text, { 0.6f, 0.6f, 0.6f, 1 });
    }

    ImGui::PushItemWidth(location.z);
    ImGui::SetCursorPos({ location.x, location.y + 3 });
    ImGui::TextUnformatted(label.c_str());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", description.c_str());

    ImGui::SetCursorPos({ location.x + 170, location.y });

    if (is_int)
    {
        int value_as_int = static_cast<int>(value);
        value_changed = ImGui::SliderInt(("##" + name).c_str(), &value_as_int, static_cast<int>(range.min), static_cast<int>(range.max), "%.0f");
        value = static_cast<float>(value_as_int);
    }
    else
    {
        value_changed = ImGui::SliderFloat(("##" + name).c_str(), &value, range.min, range.max, "%.3f", 1.0f);
    }

    ImGui::PopItemWidth();

    if (!enabled)
    {
        ImGui::PopStyleColor(3);
    }
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);
    return value_changed;
}


bool filter_slider_ui::is_all_integers(const rs2::option_range& range)
{
    const auto is_integer = [](float f)
    {
        return (fabs(fmod(f, 1)) < std::numeric_limits<float>::min());
    };

    return is_integer(range.min) && is_integer(range.max) &&
           is_integer(range.def) && is_integer(range.step);
}

void render_ui(float w, float h, std::vector<filter_options>& filters)
{
// Flags for displaying ImGui window
static const int flags = ImGuiWindowFlags_NoCollapse
                         | ImGuiWindowFlags_NoScrollbar
                         | ImGuiWindowFlags_NoSavedSettings
                         | ImGuiWindowFlags_NoTitleBar
                         | ImGuiWindowFlags_NoResize
                         | ImGuiWindowFlags_NoMove;

ImGui_ImplGlfw_NewFrame(1);
ImGui::SetNextWindowSize({ w, h });
ImGui::Begin("app", nullptr, flags);

// Using ImGui library to provide slide controllers for adjusting the filter options
const float offset_x = w / 4;
const int offset_from_checkbox = 120;
float offset_y = h / 2;
float elements_margin = 45;
for (auto& filter : filters)
{
// Draw a checkbox per filter to toggle if it should be applied
ImGui::SetCursorPos({ offset_x, offset_y });
ImGui::PushStyleColor(ImGuiCol_CheckMark, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
bool tmp_value = filter.is_enabled;
ImGui::Checkbox(filter.filter_name.c_str(), &tmp_value);
filter.is_enabled = tmp_value;
ImGui::PopStyleColor();

if (filter.supported_options.size() == 0)
{
offset_y += elements_margin;
}
// Draw a slider for each of the filter's options
for (auto& option_slider_pair : filter.supported_options)
{
filter_slider_ui& slider = option_slider_pair.second;
if (slider.render({ offset_x + offset_from_checkbox, offset_y, w / 4 }, filter.is_enabled))
{
filter.filter.set_option(option_slider_pair.first, slider.value);
}
offset_y += elements_margin;
}
}

ImGui::End();
ImGui::Render();
}

void update_data(rs2::frame_queue& data, rs2::frame& colorized_depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map)
{
    frame f;
    if(data.poll_for_frame(&f))
    {
        points = pc.calculate(f);
        colorized_depth = color_map.process(f);
        pc.map_to(colorized_depth);
        view.tex.upload(colorized_depth);
    }
}

filter_options::filter_options(const std::string name, rs2::filter& flt) :
        filter_name(name),
        filter(flt),
        is_enabled(true)
{
    const std::array<rs2_option, 5> possible_filter_options = {
            RS2_OPTION_FILTER_MAGNITUDE,
            RS2_OPTION_FILTER_SMOOTH_ALPHA,
            RS2_OPTION_MIN_DISTANCE,
            RS2_OPTION_MAX_DISTANCE,
            RS2_OPTION_FILTER_SMOOTH_DELTA
    };

    //Go over each filter option and create a slider for it
    for (rs2_option opt : possible_filter_options)
    {
        if (flt.supports(opt))
        {
            rs2::option_range range = flt.get_option_range(opt);
            supported_options[opt].range = range;
            supported_options[opt].value = range.def;
            supported_options[opt].is_int = filter_slider_ui::is_all_integers(range);
            supported_options[opt].description = flt.get_option_description(opt);
            std::string opt_name = flt.get_option_name(opt);
            supported_options[opt].name = name + "_" + opt_name;
            std::string prefix = "Filter ";
            supported_options[opt].label = opt_name;
        }
    }
}

filter_options::filter_options(filter_options&& other) :
        filter_name(std::move(other.filter_name)),
        filter(other.filter),
        supported_options(std::move(other.supported_options)),
        is_enabled(other.is_enabled.load())
{
}
