#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Char.h"
#include "ford_project/haptic_msg.h"

#include "pmd.h"
#include "usb-3100.h"

typedef enum
{
    State_Haptic_Idle,
    State_Haptic_Render,
    State_Haptic_Pause
} haptic_control_states;

class HapticController
{
public:
    // constructor
    HapticController();

    // destructor
    ~HapticController();

    // main update function
    void update();

    // initialization
    int init();

private:
    // node handler
    ros::NodeHandle nh;

    // subscriber and publisher
    ros::Subscriber haptic_control_sub;

    // output channels and board number
    int a_out0;
    int a_out1;

    hid_device *hid_3101;

    // control variables
    double t_step;
    double t_ramp;
    double t_state;
    double t_render;
    double t_state_start;
    double t_vib_start;

    // haptic variables
    int dir;
    int repetition;
    double period_render;  // in secs
    double period_pause;
    double amp_max0;
    double amp_max1;

    // state variables
    haptic_control_states state;
    int set_state;
    int vib_state;

    // callback functions
    void hapticCallback(const ford_project::haptic_msg::ConstPtr& msg);

    // other functions
    void render(double amp_max1, double amp_max2);
    double calc_amp(double amp_max, double t_now);
};
