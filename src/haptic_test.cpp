////////// INCLUDLES //////////
#include <iostream>
#include <math.h>

#include "826api.h"
#include "ros/ros.h"

////////// DEFINES //////////
#define COUNTER		CNTR_0A		// Counter A (and implicitly, B) channel to use.
#define GATE_TIME	1000		// Gate time in milliseconds.
#define DAC_VSCALAR 32767		// Binary-to-volts scalar for DAC

#define pi 3.14159

#define SAMPLE_RATE (1000)

// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's bool

// output channel and board
int ao1 = 0;
uint board = 0;

int errcode     = S826_ERR_OK;

// sine wave frequency
double freq = 50.0;

// error handle macro
#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

///////// FUNCTION PROTOTYPES ///////////
int InitSensoray();

///////// FUNCTION IMPLEMENTATIONS ///////////
int InitSensoray()
{
    int boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards

    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0)
        printf("TARGET BOARD NOT FOUND\n");         // driver didn't find board you want to use
    else
    {
        X826( S826_DacRangeWrite(board, ao1, S826_DAC_SPAN_5_5, 0)   );      // program dac output range: -10V to +10V

        X826( S826_DacDataWrite(board, ao1, (uint)(DAC_VSCALAR), 0)  );

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

    // program dac output range: -10V to +10V
    S826_DacRangeWrite(board, 0, S826_DAC_SPAN_5_5, 0);

    return 0;
}

int main(int argc, char** argv)
{
    // initialize node
    ros::init(argc, argv, "haptic_test_node");
    ros::NodeHandle nh;

    // initialize the sensoray board
    InitSensoray();

    std::cout << "board initialized!" << std::endl;

    // "haptic" loop for controlling motor
    double t_start = ros::Time::now().toSec();

    ros::Rate rate(1000);
    while (ros::ok())
    {
        double t_now = ros::Time::now().toSec() - t_start;
        double amp = std::sin(2.0 * pi * freq * t_now);

        uint dacval = (uint)(DAC_VSCALAR + DAC_VSCALAR * amp / 5.0);
        X826( S826_DacDataWrite(board, ao1, dacval, 0) );

        rate.sleep();
    }

    // Program all analog outputs to zero volts.
    S826_DacDataWrite(board, ao1, (uint)(DAC_VSCALAR), 0);

    S826_SystemClose();
}
