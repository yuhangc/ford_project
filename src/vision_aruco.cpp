#include "vision_aruco.h"
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

#define sqrt_12 0.70710678118
#define PI 3.141592653589793
// #define DRAW_MARKER

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

    ros::param::param<float>("~marker_size", this->m_marker_size, 0.19);

    ros::param::param<bool>("~recore_marker_pose", this->m_flag_record_marker_pose, false);

    // initialize aruco trackers
    this->m_detector.setThresholdParams(7, 7);
    this->m_detector.setThresholdParamRange(2, 0);
    this->m_detector.setDictionary(dict);

    // get camera info
    this->m_cam_param.readFromXMLFile(camera_info_file);

    // initialize publishers and subscribers
    this->m_human_pose_pub = this->nh.advertise<geometry_msgs::Pose2D>("tracking/human_pos2d", 1);
    this->m_human_vel_pub = this->nh.advertise<geometry_msgs::Vector3>("tracking/human_vel2d", 1);
    this->m_tracking_status_pub = this->nh.advertise<std_msgs::String>("tracking/status", 1);
    this->m_camera_rgb_sub = this->nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1,
                                                                    &ArucoTracker::camera_rgb_callback, this);

    // open file for recording marker poses
    if (this->m_flag_record_marker_pose) {
        this->m_data_file.open("/home/yuhangche/Desktop/data/marker_calibration/test.txt");
    }

    // initialize the marker offset matrices
    Eigen::Matrix4d t_rot;
    t_rot << sqrt_12,   -sqrt_12,   0,  0.172,
             0,         0,          1,  0,
             -sqrt_12,  -sqrt_12,   0,  -0.074,
             0,         0,          0,  1;
    this->m_marker_offset.push_back(t_rot);

    t_rot << 1, 0,  0,  0,
             0, 0,  1,  0,
             0, -1, 0,  0,
             0, 0,  0,  1;
    this->m_marker_offset.push_back(t_rot);

    t_rot << sqrt_12,   sqrt_12,    0,  -0.17,
             0,         0,          1,  0,
             sqrt_12,   -sqrt_12,   0,  -0.08,
             0,         0,          0,  1;
    this->m_marker_offset.push_back(t_rot);

    // initialize marker "standard" pose
    this->m_marker_rot_std << 1,  0,  0,
                              0, -1,  0,
                              0,  0, -1;

    // initialize markers to be not seen
    for (int i = 0; i < NUM_MARKER; i++) {
        this->m_marker_detected[i] = false;
    }

    // tolerance and max iteration for averaging rotation
    this->m_rot_tol = 0.01;
    this->m_rot_max_iter = 100;
}

// ============================================================================
// distructor
ArucoTracker::~ArucoTracker()
{
    if (this->m_flag_record_marker_pose) {
        this->m_data_file.close();
    }
}

// ============================================================================
// main update
void ArucoTracker::update()
{
    // detect the markers
    this->m_markers = this->m_detector.detect(this->m_image_input, this->m_cam_param, this->m_marker_size);

    if (this->m_flag_record_marker_pose) {
        double t_pos[3];
        double t_rot[4];

        // output the pose of each marker and save to file
        for (int i = 0; i < this->m_markers.size(); i++) {
            // obtain position and quaternions
            this->m_markers[i].OgreGetPoseParameters(t_pos, t_rot);

            std::cout << i << ", " << this->m_markers[i].Rvec << std::endl;

            this->m_data_file << i << ", " << t_pos[0]
                              << ", " << t_pos[1]
                              << ", " << t_pos[2]
                              << ", " << t_rot[0]
                              << ", " << t_rot[1]
                              << ", " << t_rot[2]
                              << ", " << t_rot[3] << std::endl;
        }
    }

#ifdef DRAW_MARKER
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
#endif

    // check which markers are seen
    for (int i = 0; i < NUM_MARKER; i++) {
        this->m_marker_detected[i] = false;
    }

    int t_marker_id;
    for (int i = 0; i < this->m_markers.size(); i++) {
        switch(this->m_markers[i].id) {
        case 10:
            t_marker_id = 0;
            break;
        case 20:
            t_marker_id = 1;
            break;
        case 30:
            t_marker_id = 2;
            break;
        }

        this->m_marker_detected[t_marker_id] = true;
    }

    // clear the marker rotation and position vector
    this->m_marker_pos.clear();
    this->m_marker_rot.clear();

    // convert axis-angle representation to rotation matrix
    int t_num_marker_detected = 0;
    for (int i = 0; i < NUM_MARKER; i++) {
        Eigen::Matrix3d t_rot;
        Eigen::Vector3d t_pos;

        if (this->m_marker_detected[i]) {
            Eigen::Vector3d t_angle_axis;
            t_angle_axis << this->m_markers[t_num_marker_detected].Rvec.at<float>(0, 0),
                            this->m_markers[t_num_marker_detected].Rvec.at<float>(1, 0),
                            this->m_markers[t_num_marker_detected].Rvec.at<float>(2, 0);
            t_rot = Eigen::AngleAxisd(t_angle_axis.norm(), t_angle_axis.normalized());

            t_pos << this->m_markers[t_num_marker_detected].Tvec.at<float>(0, 0),
                     this->m_markers[t_num_marker_detected].Tvec.at<float>(1, 0),
                     this->m_markers[t_num_marker_detected].Tvec.at<float>(2, 0);

            t_num_marker_detected ++;
        }

        this->m_marker_rot.push_back(t_rot);
        this->m_marker_pos.push_back(t_pos);
    }

    // report lost if non of the markers are seen
    if (t_num_marker_detected == 0) {
        this->m_tracking_status.data = "Lost";

        // publish status
        this->m_tracking_status_pub.publish(this->m_tracking_status);

        return;
    } else {
        this->m_tracking_status.data = "Find";
    }

    // construct the body transformation matrices
    std::vector<Eigen::Matrix3d> t_body_rot;
    std::vector<Eigen::Vector3d> t_body_pos;

    for (int i = 0; i < NUM_MARKER; i++) {
        Eigen::Matrix4d t_pose;
        t_pose << this->m_marker_rot.at(i), this->m_marker_pos.at(i),
                Eigen::MatrixXd::Zero(1, 3), 1;

        t_pose = t_pose * this->m_marker_offset.at(i);
        t_body_rot.push_back(t_pose.topLeftCorner(3, 3));
        t_body_pos.push_back(t_pose.topRightCorner(3, 1));

//        std::cout << t_pose << "  " << i << std::endl;
    }

    // calculate weight for the 3 markers
    double t_marker_weight[NUM_MARKER];
    for (int i = 0; i < NUM_MARKER; i++) {
        if (!this->m_marker_detected[i]) {
            // 0 weight if not seen
            t_marker_weight[i] = 0;
        } else {
            // higher weight for the ones close to "standard" pose
            Eigen::AngleAxisd t_angle_axis(this->m_marker_rot.at(i).transpose() * this->m_marker_rot_std);
            double t_angle_diff = std::abs(std::abs(t_angle_axis.angle()) - 0.5 * PI);

            if (t_angle_diff > 0.25 * PI) {
                t_marker_weight[i] = 2.0;
            } else {
                t_marker_weight[i] = 1.0;
            }
        }
    }

    // calculate weighted average of the body orientation
    Eigen::Matrix3d t_rot_avg;
    for (int i = 0; i < NUM_MARKER; i++) {
        if (this->m_marker_detected[i]) {
            t_rot_avg = t_body_rot.at(i);
            break;
        }
    }

    double r = 1.0;
    int iter = 0;
    while ((t_num_marker_detected > 1) && (r > this->m_rot_tol) && (iter < this->m_rot_max_iter)) {
        // compute gradient
        Eigen::Vector3d r_vec = Eigen::Vector3d::Zero();
        int n = 0;

        for (int i = 0; i < NUM_MARKER; i++) {
            if (this->m_marker_detected[i]) {
                n += t_marker_weight[i];

                Eigen::AngleAxisd t_angle_axis(t_rot_avg.transpose() * t_body_rot.at(i));
                r_vec = r_vec + t_marker_weight[i] * t_angle_axis.angle() * t_angle_axis.axis();
            }
        }
        r_vec = r_vec / (double) n;

        // update average rotation
        t_rot_avg = t_rot_avg * Eigen::AngleAxisd(r_vec.norm(), r_vec.normalized());

        // update residual and iterator
        r = r_vec.norm();
        iter ++;
    }

//    std::cout << t_rot_avg << "  " << iter << std::endl;

    // calculate weighted average translation of the body
    Eigen::Vector3d t_pos_avg = Eigen::Vector3d::Zero();
    int n = 0;

    for (int i = 0; i < NUM_MARKER; i++) {
        t_pos_avg = t_pos_avg + t_marker_weight[i] * t_body_pos.at(i);
        n += t_marker_weight[i];
    }
    t_pos_avg = t_pos_avg / (double) n;

    // assign to human pose variable
    this->m_body_pose.x = t_pos_avg[0];
    this->m_body_pose.y = t_pos_avg[2];
    this->m_body_pose.theta = std::atan2(t_rot_avg(0, 1), t_rot_avg(2, 1));

//    std::cout << this->m_body_pose << std::endl;
//    std::getchar();

    // publish the tracking data
    this->m_tracking_status_pub.publish(this->m_tracking_status);
    this->m_human_pose_pub.publish(this->m_body_pose);

    //! publish 0 velocity for now
    this->m_body_vel = geometry_msgs::Vector3();
    this->m_human_vel_pub.publish(this->m_body_vel);
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
