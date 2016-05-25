#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"

#include "CArduinoDevice.h"

class IMUManager
{
public:
    // constructor
    IMUManager();

    // destructor
    ~IMUManager();

    // main update function
    void update();

private:
    // node handler
    ros::NodeHandle nh;

    // subscriber and publisher
    ros::Publisher raw_data_pub;
    ros::Publisher button_pub;
    ros::Publisher rot_pub;

    // arduino device
    CArduinoDevice* arduino;

    // raw data and orientation
    sensor_msgs::Imu imu_raw;
    geometry_msgs::Vector3 rot_raw;
    geometry_msgs::Vector3 rot_filtered;

    std_msgs::Bool button_val;
};
