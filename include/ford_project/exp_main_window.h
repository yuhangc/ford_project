#ifndef EXP_MAIN_WINDOW_H
#define EXP_MAIN_WINDOW_H

#include <fstream>
#include <vector>

#include <QMainWindow>
#include <QTimer>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "ford_project/haptic_msg.h"

namespace Ui {
class ExpMainWindow;
}

class ExpMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ExpMainWindow(QWidget *parent = 0);
    ~ExpMainWindow();

    // Initialization
    void Init();

private:
    Ui::ExpMainWindow *ui;

    // Timer for GUI update
    QTimer m_update_timer;

    // node handler
    ros::NodeHandle nh;

    // subscribers
    ros::Subscriber robot_state_sub;
    ros::Subscriber robot_odom_sub;
    ros::Subscriber cmd_vel_sub;

    ros::Subscriber tracking_status_sub;
    ros::Subscriber human_pose_sub;
    ros::Subscriber human_vel_sub;

    ros::Subscriber imu_acc_sub;
    ros::Subscriber imu_gyro_sub;
    ros::Subscriber imu_mag_sub;
    ros::Subscriber imu_tilt_sub;

    ros::Subscriber button_data_sub;
    ros::Subscriber button_event_sub;

    ros::Subscriber gesture_rec_sub;

    ros::Subscriber haptic_control_sub;

    ros::Subscriber sys_msg_sub;

    // publishers
    ros::Publisher set_robot_state_pub;
    ros::Publisher set_condition_pub;
    ros::Publisher set_stuck_mode_pub;
    ros::Publisher haptic_control_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher reverse_mapping_pub;

    // set state
    std_msgs::Int8 set_state;

    // set condition
    int set_condition;
    int set_stuck_mode;

    // haptic signal
    ford_project::haptic_msg haptic_signal;
    ford_project::haptic_msg haptic_signal_input;

    // button data and event
    bool button_data;
    int button_event;

    // set velocities
    double vel_inc_limit_lin;
    double vel_inc_limit_ang;
    geometry_msgs::Twist cmd_vel_limits;
    geometry_msgs::Twist cmd_vel_goal;
    geometry_msgs::Twist cmd_vel;

    // flag for reverse velocity mapping
    double flag_reverse_mapping;

    // variables for haptic pre study
    int haptic_trial_count;
    int num_trials;
    int dir_user;
    std::vector<int> dir_trial;

    std::string protocal_file_path;
    std::ofstream haptic_data_file;

    // variable for data saving
    std::string data_file_path;
    std::ofstream data_file;
    bool flag_start_data_saving;

    double time_start_data_saving;
    int robot_state;

    int file_count;

    // callback functions
    void robot_state_callback(const std_msgs::Int8::ConstPtr& state_msg);
    void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);

    void tracking_status_callback(const std_msgs::String::ConstPtr& tracking_status_msg);
    void human_pose_callback(const geometry_msgs::Pose2D::ConstPtr& human_pose_msg);
    void human_vel_callback(const geometry_msgs::Vector3::ConstPtr& human_vel_msg);

    void imu_acc_callback(const geometry_msgs::Vector3::ConstPtr& acc_msg);
    void imu_gyro_callback(const geometry_msgs::Vector3::ConstPtr& gyro_msg);
    void imu_mag_callback(const geometry_msgs::Vector3::ConstPtr& mag_msg);
    void imu_tilt_callback(const geometry_msgs::Vector3::ConstPtr& tilt_msg);
    void button_data_callback(const std_msgs::Bool::ConstPtr& button_msg);
    void button_event_callback(const std_msgs::Int8::ConstPtr& button_event_msg);

    void gesture_rec_callback(const std_msgs::Int8::ConstPtr& gesture_msg);

    void haptic_control_callback(const ford_project::haptic_msg::ConstPtr& haptic_msg);

    void sys_msg_callback(const std_msgs::String::ConstPtr& sys_msg);

private slots:
    void UpdateGUIInfo();
    void on_combo_set_state_currentIndexChanged(int index);
    void on_button_set_state_clicked();
    void on_spinBox_vel_lin_valueChanged(double arg1);
    void on_spinBox_vel_ang_valueChanged(double arg1);
    void on_button_tele_forward_pressed();
    void on_combo_haptic_dir_currentIndexChanged(int index);
    void on_spinBox_haptic_rep_valueChanged(int arg1);
    void on_spinBox_haptic_on_valueChanged(double arg1);
    void on_spinBox_haptic_off_valueChanged(double arg1);
    void on_button_send_haptic_clicked();
    void on_combo_haptic_mag_currentIndexChanged(int index);
    void on_button_start_condition_clicked();
    void on_button_stop_record_clicked();
    void on_button_reverse_mapping_clicked();
    void on_button_load_protocal_clicked();
    void on_button_play_haptic_clicked();
    void on_button_next_trial_clicked();
    void on_button_record_forward_clicked();
    void on_button_record_backward_clicked();
    void on_button_record_left_clicked();
    void on_button_record_right_clicked();
    void on_combo_stuck_mode_currentIndexChanged(int index);
    void on_combo_tether_condition_currentIndexChanged(int index);
};

#endif // EXP_MAIN_WINDOW_H
