#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

#include "Eigen/Dense"

class VideoProcessor
{
public:
    // constructor
    VideoProcessor();

    // destructor
    ~VideoProcessor();

    // initialization
    void init(std::string file_path);
    void get_path(std::string file_path);
    void close_all();

    // update functions
    bool get_frame();
    void get_human_pos();
    void get_robot_pos();

private:
    // Aruco marker detector
    aruco::MarkerDetector m_detector;
    std::vector<aruco::Marker> m_markers;

    // orientation and position of the markers
    std::vector<Eigen::Matrix3d> m_marker_rot;
    std::vector<Eigen::Vector3d> m_marker_pos;

    // camera parameters
    aruco::CameraParameters m_cam_param;

    // marker size
    float m_marker_size;

    // marker id for robot
    int m_marker_id_robot;

    // contour area threshold for reporting lost
    float m_min_area_found;

    // flags for finding human and robot
    bool m_flag_human_found;
    bool m_flag_robot_found;

    // human and robot trajectory
    std::vector<geometry_msgs::Pose2D> m_human_traj;
    std::vector<geometry_msgs::Pose2D> m_robot_traj;

    // camera infos
    int m_width;
    int m_height;

    // video input
    cv::VideoCapture m_video_capture;

    // frame count and rate
    int m_frame_count;
    int m_frame_rate;

    // image from camera
    cv::Mat m_image_input;

    // data recording streams
    std::ofstream m_path_data;
    std::ofstream m_human_data;
    std::ofstream m_robot_data;
};
