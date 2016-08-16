#include "imu_manager.h"
#include <string>
#include <iostream>
#include <sstream>

// ============================================================================
// constructor
IMUManager::IMUManager()
{
    // initialize publisher
    this->acc_data_pub = this->nh.advertise<geometry_msgs::Vector3>("human_input/acc_raw", 1);
    this->gyro_data_pub = this->nh.advertise<geometry_msgs::Vector3>("human_input/gyro_raw", 1);
    this->meg_data_pub = this->nh.advertise<geometry_msgs::Vector3>("human_input/mag_raw", 1);
    this->button_pub = this->nh.advertise<std_msgs::Bool>("human_input/button", 1);

    // initialize the arduino device
    this->arduino = new CArduinoDevice("/dev/ttyACM0",CArduinoDevice::BAUD_115200);
    if (!this->arduino->connect()) {
        ROS_ERROR("Cannot connect to input device!");
    } else {
        ROS_INFO("Input device connected");
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

    if (read_n > 10) {
        ss >> button_data.data;
        ss.ignore(2);
        ss >> acc_data.x;
        ss.ignore(2);
        ss >> acc_data.y;
        ss.ignore(2);
        ss >> acc_data.z;
        ss.ignore(2);
        ss >> gyro_data.x;
        ss.ignore(2);
        ss >> gyro_data.y;
        ss.ignore(2);
        ss >> gyro_data.z;
        ss.ignore(2);
        ss >> meg_data.x;
        ss.ignore(2);
        ss >> meg_data.y;
        ss.ignore(2);
        ss >> meg_data.z;

        this->gyro_data_pub.publish(this->gyro_data);
        this->acc_data_pub.publish(this->acc_data);
        this->meg_data_pub.publish(this->meg_data);
        this->button_pub.publish(this->button_data);

//        std::cout << this->gyro_data << std::endl;
//        std::cout << this->acc_data << std::endl;
//        std::cout << this->meg_data << std::endl;
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
