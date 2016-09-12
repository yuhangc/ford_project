#include <fstream>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

#include "Eigen/Dense"

#define NUM_MARKER 3

class ArucoTracker
{
public:
    // constructor
    ArucoTracker();

    // destructor
    ~ArucoTracker();

    // main update function
    void update();

private:
    // node handler
    ros::NodeHandle nh;

    // subscriber and publisher
    ros::Subscriber m_camera_rgb_sub;
    ros::Subscriber m_camera_depth_sub;
    ros::Publisher m_human_pose_pub;
    ros::Publisher m_human_vel_pub;
    ros::Publisher m_tracking_status_pub;

    // Aruco marker detector
    aruco::MarkerDetector m_detector;
    std::vector<aruco::Marker> m_markers;

    // orientation and position of the markers
    std::vector<Eigen::Matrix3d> m_marker_rot;
    std::vector<Eigen::Vector3d> m_marker_pos;

    // offsets of the markers to human body
    std::vector<Eigen::Matrix4d> m_marker_offset;

    // wether the marker is seen
    bool m_marker_detected[NUM_MARKER];

    // detected body pose and velocity
    geometry_msgs::Pose2D m_body_pose;
    geometry_msgs::Pose2D m_body_pose_last;
    geometry_msgs::Vector3 m_body_vel;
    geometry_msgs::Vector3 m_body_vel_last;

    // velocity filter coefficient
    float m_vel_filter_alpha;

    // time interval for discretization
    double m_dt;

    // tracking status
    std_msgs::String m_tracking_status;

    // camera parameters
    aruco::CameraParameters m_cam_param;

    // front and back marker id
    int m_marker_id_front;
    int m_marker_id_back;

    // marker size
    float m_marker_size;

    // "standard" pose of marker
    Eigen::Matrix3d m_marker_rot_std;

    // convergence tolerance and max iteration
    double m_rot_tol;
    double m_rot_max_iter;

    // contour area threshold for reporting lost
    float m_min_area_found;

    // depth data scale and depth data offset
    float m_depth_scale;
    float m_depth_offset;
    float m_depth_cap_min;

    // camera infos
    int m_width;
    int m_height;
    float m_fov;
    float m_fw;
    float m_fh;

    // image from camera
    cv::Mat m_image_input;
    cv::Mat m_depth_input;

    // data recording stream
    std::ofstream m_data_file;

    // flags
    bool m_flag_rgb_image_received;
    bool m_flag_depth_image_received;
    bool m_flag_record_marker_pose;

    // callback functions
    void camera_rgb_callback(const sensor_msgs::ImageConstPtr& image_msg);
    void camera_depth_callback(const sensor_msgs::ImageConstPtr& image_msg);

    // other functions
    void report_lost();
};
