#include <vector>
#include <string>
#include <deque>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int8.h"
#include "eigen3/Eigen/Dense"

typedef enum
{
    State_Detection_Idle,
    State_Detection_Start,
    State_Detection_Debounce
} detection_states;

class GestureDTW
{
public:
    // constructor
    GestureDTW();

    // destructor
    ~GestureDTW();

    // main update function
    void update();

private:
    // some "constants"
    int num_gestures;
    int max_window_size;
    int dtw_window;
    std::vector<Eigen::MatrixXd> gesture_temps;
    std::vector<double> rej_thresh;
    std::vector<double> rej_corr;

    // node handler
    ros::NodeHandle nh;

    // subscriber and publisher
    ros::Subscriber acc_data_sub;
    ros::Publisher gesture_pub;

    // acceleration data and history
    Eigen::Vector3d acc_raw;
    std::deque<Eigen::Vector3d> acc_filtered;

    // state for gesture module
    detection_states state;

    // recognized gesture
    std_msgs::Int8 gesture_id;

    // debounce time for gesture detection
    double time_debounce;
    double time_gesture_start;

    // flags
    bool flag_gesture;
    bool set_gesture_detect_start;

    // callback functions
    void acc_data_callback(const geometry_msgs::Vector3::ConstPtr& msg);

    // functions
    void load_data(std::string file_path);
    void add_filtered_acc(void);
    void gesture_matching(void);
    double dist(Eigen::VectorXd v1, Eigen::VectorXd v2);
};
