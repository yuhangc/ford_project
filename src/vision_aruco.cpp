#include "vision_aruco.h"
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

// ============================================================================
// constructor
ArucoTracker::ArucoTracker()
{
    std::string dict;
    std::string camera_info_file;
    // get parameters
    ros::param::param<std::string>("~dictionary", dict, "ARUCO");
    ros::param::param<std::string>("~camera_info_file", camera_info_file, "camera_calibration.yml");

    ros::param::param<int>("~marker_id_front", this->m_marker_id_front, 10);
    ros::param::param<int>("~marker_id_back", this->m_marker_id_back, 20);

    ros::param::param<float>("~marker_size", this->m_marker_size, 100);

    // initialize aruco trackers
    this->m_detector.setThresholdParams(7, 7);
    this->m_detector.setThresholdParamRange(2, 0);
    this->m_detector.setDictionary(dict);

    // get camera info
//    std::cout << "1" << std::endl;
    this->m_cam_param.readFromXMLFile(camera_info_file);
//    std::cout << "2" << std::endl;

    // initialize publishers and subscribers
    this->m_human_pose_pub = this->nh.advertise<std_msgs::Float32>("tracking/human_orientation", 1);
    this->m_camera_rgb_sub = this->nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1,
                                                                    &ArucoTracker::camera_rgb_callback, this);
}

// ============================================================================
// distructor
ArucoTracker::~ArucoTracker()
{
}

// ============================================================================
// main update
void ArucoTracker::update()
{
    // detect the markers
    this->m_markers = this->m_detector.detect(this->m_image_input, this->m_cam_param, this->m_marker_size);

    // draw marker boundaries
    cv::Mat t_output_image;
    this->m_image_input.copyTo(t_output_image);

    for (int i = 0; i < this->m_markers.size(); i++) {
        this->m_markers[i].draw(t_output_image, cv::Scalar(0, 0, 255), 1);
    }

    // draw cubes on the marker
    if (this->m_cam_param.isValid() && this->m_marker_size > 0) {
        for (int i = 0; i < this->m_markers.size(); i++) {
            aruco::CvDrawingUtils::draw3dCube(t_output_image, this->m_markers[i], this->m_cam_param);
            aruco::CvDrawingUtils::draw3dAxis(t_output_image, this->m_markers[i], this->m_cam_param);
        }
    }

    // display the image
    cv::imshow("test", t_output_image);
    cv::waitKey(3);
}

// ============================================================================
void ArucoTracker::camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    this->m_image_input = cv_bridge::toCvCopy(image_msg)->image;
    this->m_flag_image_received = true;

    this->update();
}

// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_aruco");
    ArucoTracker aruco_tracker;

//    ros::Duration(0.1).sleep();

//    ros::Rate loop_rate(10);
//    while (!ros::isShuttingDown())
//    {
//        std::cout << "loop!" << std::endl;
//        ros::spinOnce();
//        aruco_tracker.update();
//        loop_rate.sleep();
//    }
    ros::spin();
}
