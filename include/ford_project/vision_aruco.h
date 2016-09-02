#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"

#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

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
    ros::Publisher m_human_pose_pub;

    // Aruco marker detector
    aruco::MarkerDetector m_detector;
    std::vector<aruco::Marker> m_markers;

    // camera parameters
    aruco::CameraParameters m_cam_param;

    // front and back marker id
    int m_marker_id_front;
    int m_marker_id_back;

    // marker size
    float m_marker_size;

    // image from camera
    cv::Mat m_image_input;

    // flag for image update
    bool m_flag_image_received;

    // callback functions
    void camera_rgb_callback(const sensor_msgs::ImageConstPtr& image_msg);
};
