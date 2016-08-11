//------------------------------- Include ----------------------------------
#include <iostream>
#include <math.h>

#include "haptic_test_PCI.h"

//------------------------------- Defines ----------------------------------
#define DAC_VSCALAR 32767		// Binary-to-volts scalar for DAC
#define pi 3.14159

// ============================================================================
HapticController::HapticController()
{
    this->board = 0;
    ros::param::param<int>("~analog_out_channel0", this->a_out0, 0);
    ros::param::param<int>("~analog_out_channel1", this->a_out1, 1);
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
    S826_DacDataWrite(this->board, this->a_out0, (uint)(DAC_VSCALAR), 0);
    S826_DacDataWrite(this->board, this->a_out1, (uint)(DAC_VSCALAR), 0);

    S826_SystemClose();
}

// ============================================================================
void HapticController::init()
{
    int errcode = S826_ERR_OK;

    int boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards

    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0)
        printf("TARGET BOARD NOT FOUND\n");         // driver didn't find board you want to use
    else
    {
        // set output range
        S826_DacRangeWrite(this->board, this->a_out0, S826_DAC_SPAN_5_5, 0);
        S826_DacRangeWrite(this->board, this->a_out1, S826_DAC_SPAN_5_5, 0);

        S826_DacDataWrite(this->board, this->a_out0, (uint)(DAC_VSCALAR), 0);
        S826_DacDataWrite(this->board, this->a_out1, (uint)(DAC_VSCALAR), 0);

        // Suspend this thread for 5 seconds.
        ros::Duration(5).sleep();
    }

    switch (errcode)
    {
    case S826_ERR_OK:           break;
    case S826_ERR_BOARD:        printf("Illegal board number"); break;
    case S826_ERR_VALUE:        printf("Illegal argument"); break;
    case S826_ERR_NOTREADY:     printf("Device not ready or timeout"); break;
    case S826_ERR_CANCELLED:    printf("Wait cancelled"); break;
    case S826_ERR_DRIVER:       printf("Driver call failed"); break;
    case S826_ERR_MISSEDTRIG:   printf("Missed adc trigger"); break;
    case S826_ERR_DUPADDR:      printf("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
    case S826_ERR_BOARDCLOSED:  printf("Board not open"); break;
    case S826_ERR_CREATEMUTEX:  printf("Can't create mutex"); break;
    case S826_ERR_MEMORYMAP:    printf("Can't map board"); break;
    default:                    printf("Unknown error"); break;
    }

//    // program dac output range
//    S826_DacRangeWrite(board, 0, S826_DAC_SPAN_5_5, 0);

    ROS_INFO("Sensoray board initialized!");

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
    uint dacval;

    dacval = (uint)(DAC_VSCALAR + DAC_VSCALAR * amp0 / 5.0);
    S826_DacDataWrite(this->board, this->a_out0, dacval, 0);

    dacval = (uint)(DAC_VSCALAR + DAC_VSCALAR * amp1 / 5.0);
    S826_DacDataWrite(this->board, this->a_out1, dacval, 0);
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
