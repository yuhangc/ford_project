#include "video_processing.h"

#include <string>
#include <iostream>
#include <sstream>

#include <ros/console.h>

#define sqrt_12 0.70710678118
#define PI 3.141592653589793
// #define DRAW_MARKER

// ============================================================================
// constructor
VideoProcessor::VideoProcessor()
{
    std::string dict;
    std::string camera_info_file;

    // get parameters
    ros::param::param<std::string>("~dictionary", dict, "ARUCO");
    ros::param::param<std::string>("~camera_info_file", camera_info_file, "gopro.yml");

    ros::param::param<int>("~marker_id_robot", this->m_marker_id_robot, 11);
    ros::param::param<float>("~marker_size", this->m_marker_size, 0.19);
    ros::param::param<float>("~min_area_contour_found", this->m_min_area_found, 1000);
    ros::param::param<int>("~frame_rate", this->m_frame_rate, 60);

    int t_default_id[] = {40, 50, 60, 70};
    for (int i = 0; i < NUM_MARKER_CALIBRATION; i++) {
        std::string param_name = "marker_id_calibration" + std::to_string(i);
        ros::param::param<int>(param_name, this->m_marker_id_calibration[i], t_default_id[i]);
    }

    this->m_width = 1920;
    this->m_height = 1440;

    // initialize aruco trackers
    this->m_detector.setThresholdParams(7, 7);
    this->m_detector.setThresholdParamRange(2, 0);
    this->m_detector.setDictionary(dict);

    // get camera info
    this->m_cam_param.readFromXMLFile(camera_info_file);

    // set frame count to be zero
    this->m_frame_count = 0;
}

// ============================================================================
// distructor
VideoProcessor::~VideoProcessor()
{
}

// ============================================================================
void VideoProcessor::init(std::string file_path)
{
    // initialize the data save streams
    std::string file_name = file_path;
    file_name.append("/path_data.txt");
    this->m_path_data.open(file_name);

    file_name = file_path;
    file_name.append("/calibration_data.txt");
    this->m_calibration_data.open(file_name);

    file_name = file_path;
    file_name.append("/human_data.txt");
    this->m_human_data.open(file_name);

    file_name = file_path;
    file_name.append("/robot_data.txt");
    this->m_robot_data.open(file_name);

    // open the video
    file_name = file_path;
    file_name.append("/cond1_no_haptics.MP4");
    this->m_video_capture.open(file_name);
}

// ============================================================================
void VideoProcessor::close_all()
{
    this->m_path_data.close();
    this->m_robot_data.close();
    this->m_human_data.close();
}

// ============================================================================
void VideoProcessor::calibrate(std::string file_name)
{
    this->m_image_input = cv::imread(file_name);

    aruco::CameraParameters t_cam_param = this->m_cam_param;
//    t_cam_param.CamSize = cv::Size(4000, 3000);
//    t_cam_param.CameraMatrix.at<float>(0,0) = t_cam_param.CameraMatrix.at<float>(0,0) / 2.0;
//    t_cam_param.CameraMatrix.at<float>(1,1) = t_cam_param.CameraMatrix.at<float>(0,0) / 2.0;
//    t_cam_param.CameraMatrix.at<float>(0,2) = t_cam_param.CameraMatrix.at<float>(0,0) * 2.0;
//    t_cam_param.CameraMatrix.at<float>(1,2) = t_cam_param.CameraMatrix.at<float>(0,0) * 2.0;
//    std::cout << t_cam_param.CamSize << std::endl;

    // detect the markers
    this->m_markers = this->m_detector.detect(this->m_image_input, t_cam_param, this->m_marker_size);

    // not enough marker
//    if (this->m_markers.size() != NUM_MARKER_CALIBRATION) {
//        ROS_ERROR("wrong marker number! only %d markers found", this->m_markers.size());
//        return;
//    }

    // draw the markers
    cv::Mat t_output_image;
    this->m_image_input.copyTo(t_output_image);
    for (int i = 0; i < this->m_markers.size(); i++) {
        this->m_markers[i].draw(t_output_image, cv::Scalar(0, 0, 255), 1);
        aruco::CvDrawingUtils::draw3dCube(t_output_image, this->m_markers[i], this->m_cam_param);
        aruco::CvDrawingUtils::draw3dAxis(t_output_image, this->m_markers[i], this->m_cam_param);
    }
    cv::Mat t_display;
    cv::resize(t_output_image, t_display, cv::Size(), 0.3, 0.3);
    cv::imshow("test", t_display);
    cv::waitKey(500);
    std::getchar();

    // write the positions of each marker to file
    for (int i = 0; i < this->m_markers.size(); i++) {
        this->m_calibration_data << this->m_markers[i].id << ",  "
                                 << this->m_markers[i].Tvec.at<float>(0, 0) << ",  "
                                 << this->m_markers[i].Tvec.at<float>(1, 0) << ",  "
                                 << this->m_markers[i].Tvec.at<float>(2, 0) << ",  "
                                 << this->m_markers[i].Rvec.at<float>(0, 0) << ",  "
                                 << this->m_markers[i].Rvec.at<float>(1, 0) << ",  "
                                 << this->m_markers[i].Rvec.at<float>(2, 0) << std::endl;
    }
}

// ============================================================================
void VideoProcessor::get_path(std::string file_name)
{
    cv::Mat t_image_input = cv::imread(file_name);
    cv::undistort(t_image_input, this->m_image_input, this->m_cam_param.CameraMatrix, this->m_cam_param.Distorsion);

    // color segmentation
    cv::Mat t_image_hsv;
    cv::cvtColor(this->m_image_input, t_image_hsv, CV_BGR2HSV);

    cv::Mat t_color_mask;
    cv::inRange(t_image_hsv, cv::Scalar(30, 50, 50), cv::Scalar(60, 255, 255), t_color_mask);

    // opening filter to remove small objects (false positives)
    cv::erode(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // closing to remove small holes (false negatives)
    cv::dilate(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    cv::Mat t_out_image, t_mask;
    cv::bitwise_and(this->m_image_input, this->m_image_input, t_mask, t_color_mask);
    cv::resize(t_mask, t_out_image, cv::Size(), 0.5, 0.5);
    cv::imshow("test", t_out_image);
    cv::waitKey(500);
    std::getchar();

    // record all non-zero coordinates in the mask
    cv::Mat non_zero_coords;
    cv::findNonZero(t_color_mask, non_zero_coords);

    for (int i = 0; i < non_zero_coords.total(); i++) {
        this->m_path_data << non_zero_coords.at<cv::Point>(i).x << ",  "
                          << non_zero_coords.at<cv::Point>(i).y << std::endl;
    }
}

// ============================================================================
bool VideoProcessor::get_frame()
{
    this->m_frame_count ++;
//    std::cout << this->m_frame_count << std::endl;
    ROS_ERROR("processing frame %d...", this->m_frame_count);

    if (this->m_video_capture.grab()) {
        this->m_video_capture.retrieve(this->m_image_input);
        return true;
    } else {
        return false;
    }

//    //! save the image
//    cv::imwrite("/home/yuhangche/Desktop/exp_video/environment.JPG", this->m_image_input);
}

// ============================================================================
void VideoProcessor::get_human_pos()
{
    // rectify image first
    cv::Mat t_image_input;
    cv::undistort(this->m_image_input, t_image_input, this->m_cam_param.CameraMatrix, this->m_cam_param.Distorsion);

    // color segmentation to find human
    // color segmentation
    cv::Mat t_image_hsv;
    cv::cvtColor(t_image_input, t_image_hsv, CV_BGR2HSV);

    cv::Mat t_color_mask;
    cv::inRange(t_image_hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), t_color_mask);

    // opening filter to remove small objects (false positives)
    cv::erode(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // closing to remove small holes (false negatives)
    cv::dilate(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(t_color_mask, t_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // find contours in the image
    std::vector<std::vector<cv::Point>> t_contours;
    std::vector<cv::Vec4i> t_hierarchy;

    cv::findContours(t_color_mask, t_contours, t_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // report lost if no contours found
    int num_contours = t_contours.size();
    if (num_contours == 0) {
        this->m_flag_human_found = false;
        return;
    }

    // find the areas of the contours
    double t_areas[num_contours];
    double t_max_area = 0;
    int idx;
    for (int i = 0; i < num_contours; i++) {
        t_areas[i] = cv::contourArea(t_contours.at(i));

        // find the max area contour
        if (t_areas[i] > t_max_area) {
            t_max_area = t_areas[i];
            idx = i;
        }
    }

    // report lost if max area too small
    if (t_max_area < this->m_min_area_found) {
        this->m_flag_human_found = false;
        return;
    }

//    // show the mask
//    cv::Mat t_mask;
//    cv::resize(t_color_mask, t_mask, cv::Size(), 0.5, 0.5);
//    cv::imshow("test", t_mask * 255);
//    cv::waitKey(50);

    // calculate the center of the largest contour
    cv::Moments mu = cv::moments(t_contours.at(idx));

    geometry_msgs::Pose2D t_pose;
    t_pose.x = mu.m10 / mu.m00;
    t_pose.y = mu.m01 / mu.m00;

    double t = (double) (this->m_frame_count - 1) / (double) this->m_frame_rate;

    // write to file
    // std::cout << this->m_frame_count << ",  " << t_pose.x << ",  " << t_pose.y << std::endl;
    this->m_human_data << t << ",  " << t_pose.x << ",  " << t_pose.y << std::endl;
}

// ============================================================================
void VideoProcessor::get_robot_pos()
{
    // detect the markers
    this->m_markers = this->m_detector.detect(this->m_image_input, this->m_cam_param, this->m_marker_size);

    // no markers found
    if (this->m_markers.size() == 0) {
        this->m_flag_robot_found = false;
        return;
    }

    // if found, check if id match the robot marker id
//    cv::Mat t_output_image;
//    this->m_image_input.copyTo(t_output_image);
    this->m_flag_robot_found = false;
    for (int i = 0; i < this->m_markers.size(); i++) {
        if (this->m_markers[i].id == this->m_marker_id_robot) {
            // found robot
            this->m_flag_robot_found = true;

//            // draw cube
//            this->m_markers[i].draw(t_output_image, cv::Scalar(0, 0, 255), 1);
//            aruco::CvDrawingUtils::draw3dCube(t_output_image, this->m_markers[i], this->m_cam_param);
//            aruco::CvDrawingUtils::draw3dAxis(t_output_image, this->m_markers[i], this->m_cam_param);

            // obtain position information
            geometry_msgs::Pose2D t_pose;
            t_pose.x = this->m_markers[i].Tvec.at<float>(0, 0);
            t_pose.y = this->m_markers[i].Tvec.at<float>(1, 0);

            // calculate orientation using Euler angle
            Eigen::Vector3d t_angle_axis;
            t_angle_axis << this->m_markers[i].Rvec.at<float>(0, 0),
                            this->m_markers[i].Rvec.at<float>(1, 0),
                            this->m_markers[i].Rvec.at<float>(2, 0);

            Eigen::Matrix3d t_rot;
            t_rot = Eigen::AngleAxisd(t_angle_axis.norm(), t_angle_axis.normalized());

            // reverse the y and z axis
            for (int i = 0; i < 3; i++) {
                for (int j = 1; j < 3; j++) {
                    t_rot(i, j) = -t_rot(i, j);
                }
            }

            Eigen::Vector3d ea = t_rot.eulerAngles(0, 1, 2);
            t_pose.theta = ea[2];

            // convert to [0 2*pi]
            if (t_pose.theta < 0) {
                t_pose.theta += 2 * PI;
            } else if (t_pose.theta > 2 * PI) {
                t_pose.theta -= 2 * PI;
            }

            // write to file
            double t = (double) (this->m_frame_count - 1) / (double) this->m_frame_rate;
            this->m_robot_data << t << ",  " << t_pose.x << ",  " << t_pose.y << ",  " << t_pose.theta << std::endl;

            break;
        }
    }

//    cv::Mat t_display;
//    cv::resize(t_output_image, t_display, cv::Size(), 0.5, 0.5);
//    cv::imshow("test", t_display);
//    cv::waitKey(5);
}

// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_process");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    VideoProcessor video_processor;

    video_processor.init("/home/yuhangche/Desktop/exp_video/user7");
    video_processor.calibrate("/home/yuhangche/Desktop/exp_video/user7/calibration.JPG");
    video_processor.get_path("/home/yuhangche/Desktop/exp_video/user7/calibration1.jpg");

    while (video_processor.get_frame() && !ros::isShuttingDown()) {
        video_processor.get_frame();
        video_processor.get_human_pos();
        video_processor.get_robot_pos();
    }

    video_processor.close_all();
}
