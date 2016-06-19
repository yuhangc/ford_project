#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
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
    ros::Publisher acc_data_pub;
    ros::Publisher gyro_data_pub;
    ros::Publisher meg_data_pub;

    // arduino device
    CArduinoDevice* arduino;

    // raw data
    geometry_msgs::Vector3 acc_data;
    geometry_msgs::Vector3 gyro_data;
    geometry_msgs::Vector3 meg_data;

    std_msgs::Bool button_val;
};
