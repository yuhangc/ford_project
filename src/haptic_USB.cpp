#include <iostream>
#include <math.h>

#include "haptic_USB.h"

#define pi 3.14159

// ============================================================================
HapticController::HapticController()
{
    ros::param::param<int>("~analog_out_channel0", this->a_out0, 1);
    ros::param::param<int>("~analog_out_channel1", this->a_out1, 3);
    ros::param::param<double>("~t_step", this->t_step, 0.007);
    ros::param::param<double>("~t_ramp", this->t_ramp, 0.018);

    // handler and subscribers
    ros::NodeHandle nh;
    this->haptic_control_sub = nh.subscribe<ford_project::haptic_msg>("haptic_control", 1,
                                                                      &HapticController::hapticCallback, this);
}

// ============================================================================
HapticController::~HapticController()
{
    // Program all analog outputs to zero volts.
    for (int ch = 0; ch < 4; ch ++) {
        u_int16_t dacval = volts_USB31XX(BP_10_00V, 0);
        usbAOut_USB31XX(this->hid_3101, ch, dacval, 0);
    }
}

// ============================================================================
int HapticController::init()
{
    // initialize the usb3101 board
    int ret = hid_init();
    if (ret < 0) {
        ROS_ERROR("hid_init failed with return code %d", ret);
        return -1;
    }

    if ((this->hid_3101 = hid_open(MCC_VID, USB3101_PID, NULL)) > 0) {
        ROS_INFO("USB 3101 device is found!");
    } else {
        ROS_ERROR("USB 3010 device not found!");
    }

    // configure digital IOs

    // configure analog output channels
    for (int ch = 0; ch < 4; ch ++) {
        usbAOutConfig_USB31XX(this->hid_3101, ch, BP_10_00V);   // +/- 10V
    }

    // set initial state
    this->state = State_Haptic_Idle;
    this->vib_state = 0;

    // get an initial time
    this->t_state_start = ros::Time::now().toSec();
    this->t_vib_start = ros::Time::now().toSec();
}

// ============================================================================
void HapticController::hapticCallback(const ford_project::haptic_msg::ConstPtr& msg)
{
    // get message data
    this->dir = msg->direction;
    this->repetition = msg->repetition;
    this->period_render = msg->period_render;
    this->period_pause = msg->period_pause;

    // set magnitudes based on direction
    switch (this->dir)
    {
    case 0:
        // left, west
        this->amp_max0 = 1.0;
        this->amp_max1 = 0;
        break;
    case 1:
        // right, east
        this->amp_max0 = -1.0;
        this->amp_max1 = 0;
        break;
    case 2:
        // up, north
        this->amp_max0 = 0;
        this->amp_max1 = 1.0;
        break;
    case 3:
        // down, south
        this->amp_max0 = 0;
        this->amp_max1 = -1.0;
        break;
    case 4:
        // upleft, NW
        this->amp_max0 = 0.707;
        this->amp_max1 = 0.707;
        break;
    case 5:
        // upright, NE
        this->amp_max0 = -0.707;
        this->amp_max1 = 0.707;
        break;
    case 6:
        // downleft, SW
        this->amp_max0 = 0.707;
        this->amp_max1 = -0.707;
        break;
    case 7:
        // downright, SE
        this->amp_max0 = -0.707;
        this->amp_max1 = -0.707;
        break;
    }
    this->amp_max0 *= msg->amplitude;
    this->amp_max1 *= msg->amplitude;

    // set state
    this->set_state = 1;
}

// ============================================================================
void HapticController::update()
{
    this->t_state = ros::Time::now().toSec();

    // state machine
    switch (this->state)
    {
    case State_Haptic_Idle:

        this->render(0, 0, 0);
        // check if set to render
        if (this->set_state == 1) {
            // reset the set state variable
            this->set_state = 0;

            this->state = State_Haptic_Render;
            this->t_state_start = ros::Time::now().toSec();
            ROS_INFO("Move to state Render!");
        }
        break;
    case State_Haptic_Render:
        this->render(this->amp_max0, this->amp_max1, this->t_state - this->t_vib_start);

        // check if time out
        if (this->t_state - this->t_state_start >= this->period_render) {
            this->repetition -= 1;
            if (this->repetition <= 0) {
                // set back to idle
                this->state = State_Haptic_Idle;
                ROS_INFO("Move to state Idle!");
            } else {
                // set to pause
                this->state = State_Haptic_Pause;
                this->t_state_start = ros::Time::now().toSec();
                ROS_INFO("Move to state Pause!");
            }
        }
        break;
    case State_Haptic_Pause:
        this->render(0, 0, 0);

        // check if time out
        if (this->t_state - this->t_state_start >= this->period_pause) {
            this->state = State_Haptic_Render;
            this->t_state_start = ros::Time::now().toSec();
            ROS_INFO("Move to state Render!");
        }
    }
}

// ============================================================================
void HapticController::render(double amp_max0, double amp_max1, double t_now)
{
    // calculate desired amplitude
    double amp0, amp1;

    if (this->vib_state == 0) {
        // calculate ramp for both actuators
        if (t_now > this->t_ramp) {
            this->vib_state = 1;
            this->t_vib_start = ros::Time::now().toSec();
        } else {
            amp0 = (-2.0 * t_now / this->t_ramp + 1.0) * amp_max0;
            amp1 = (-2.0 * t_now / this->t_ramp + 1.0) * amp_max1;
        }
    } else {
        // step the amplitude
        if (t_now > this->t_step) {
            this->vib_state = 0;
            this->t_vib_start = ros::Time::now().toSec();
        } else {
            amp0 = amp_max0;
            amp1 = amp_max1;
        }
    }

    // send signal to sensoray
    int errcode;
    u_int16_t dacval;

    dacval = volts_USB31XX(BP_10_00V, amp0);
    usbAOut_USB31XX(this->hid_3101, this->a_out0, dacval, 0);

    ros::Duration(0.000001).sleep();

    dacval = volts_USB31XX(BP_10_00V, amp1);
    usbAOut_USB31XX(this->hid_3101, this->a_out1, dacval, 0);
}

// ============================================================================
int main(int argc, char** argv)
{
    // initialize node
    ros::init(argc, argv, "haptic_test_node");

    // a haptic control object
    HapticController controller = HapticController();
    controller.init();

    ros::Rate rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        controller.update();
        rate.sleep();
    }
}
