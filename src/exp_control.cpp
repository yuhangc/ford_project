#include <iostream>
#include <QApplication>

#include "exp_main_window.h"

// ============================================================================
int main(int argc, char** argv)
{
    // initialize node
    ros::init(argc, argv, "experiment_control");

    // create a QApplication
    QApplication a(argc, argv);

    // create a main window object
    ExpMainWindow main_window;
    main_window.Init();

    // show the main window
    main_window.show();

    // exit
    return a.exec();

//    // loop until shut down
//    ros::Rate loop_rate(50);
//    while (!ros::isShuttingDown()) {
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
}
