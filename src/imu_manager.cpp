#include "imu_manager.h"
#include <string>
#include <iostream>
#include <sstream>

// ============================================================================
// constructor
IMUManager::IMUManager()
{
    // initialize publisher
    this->raw_data_pub = this->nh.advertise<sensor_msgs::Imu>("human_input/imu_raw", 100);
    this->button_pub = this->nh.advertise<std_msgs::Bool>("human_input/button", 100);
    this->rot_pub = this->nh.advertise<geometry_msgs::Vector3>("human_input/rotation", 100);

    // initialize the arduino device
    this->arduino = new CArduinoDevice("/dev/ttyACM0",CArduinoDevice::BAUD_115200);
    if (!this->arduino->connect()) {
        ROS_ERROR("Cannot connect to input device!");
    } else {
        ROS_INFO("Input deivce connected");
    }
}

// ============================================================================
// distructor
IMUManager::~IMUManager()
{
    delete  this->arduino;
}

// ============================================================================
// main update
void IMUManager::update()
{
    if (!this->arduino->isConnected()) {
        ROS_WARN("Input device not connected! Try to connect again");
        if (this->arduino->connect()) {
            ROS_INFO("Input device connected");
        } else {
            ROS_ERROR("Cannot connect to input device!");
            return;
        }
    }

    std::string message;
    int read_n = this->arduino->read(message);

    std::stringstream ss(message);
//    std::cout << read_n << "  " << message << std::endl;

    if (read_n > 10) {
        // process the message to imu raw
//        char* pch;
//        pch = std::strtok(msg, " ,");
//        this->imu_raw.header.stamp.fromSec(std::atof(pch)/1000.0);
//        pch = std::strtok(NULL, " ,");
//        this->imu_raw.angular_velocity.x = std::atof(pch);
//        pch = std::strtok(NULL, " ,");
//        this->imu_raw.angular_velocity.y = std::atof(pch);
//        pch = std::strtok(NULL, " ,");
//        this->imu_raw.angular_velocity.z = std::atof(pch);
//        pch = std::strtok(NULL, " ,");
//        this->imu_raw.linear_acceleration.x = std::atof(pch);
//        pch = std::strtok(NULL, " ,");
//        this->imu_raw.linear_acceleration.y = std::atof(pch);
//        pch = std::strtok(NULL, " ,");
//        this->imu_raw.linear_acceleration.z = std::atof(pch);

        double t;
        ss >> t;
        ss.ignore(2);
        ss >> imu_raw.angular_velocity.x;
        ss.ignore(2);
        ss >> imu_raw.angular_velocity.y;
        ss.ignore(2);
        ss >> imu_raw.angular_velocity.z;
        ss.ignore(2);
        ss >> imu_raw.linear_acceleration.x;
        ss.ignore(2);
        ss >> imu_raw.linear_acceleration.y;
        ss.ignore(2);
        ss >> imu_raw.linear_acceleration.z;

        this->raw_data_pub.publish(this->imu_raw);
//        std::cout << this->imu_raw << std::endl;

        // convert to roll and pitch
        float x = this->imu_raw.linear_acceleration.x;
        float y = this->imu_raw.linear_acceleration.y;
        float z = this->imu_raw.linear_acceleration.z;
        this->rot_raw.x = std::atan2(y, z); // * 180 / 3.14159;
        this->rot_raw.y = std::atan2(x, std::sqrt(y*y+z*z)); // * 180 / 3.14159;

        this->rot_pub.publish(this->rot_raw);
//        std::cout << rot_raw << std::endl;

        this->button_val.data = true;
        this->button_pub.publish(this->button_val);
    } else {
//        this->button_val.data = false;
//        this->button_pub.publish(this->button_val);
    }
}

// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_manager");
    IMUManager imu_manager;

    ros::Rate loop_rate(500);
    while (!ros::isShuttingDown())
    {
        imu_manager.update();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
