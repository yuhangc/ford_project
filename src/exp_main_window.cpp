#include <QShortcut>

#include "exp_main_window.h"
#include "ui_exp_main_window.h"

ExpMainWindow::ExpMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ExpMainWindow)
{
    ui->setupUi(this);
}

// ============================================================================
ExpMainWindow::~ExpMainWindow()
{
    delete ui;
}

// ============================================================================
void ExpMainWindow::Init()
{
    // Initialize the subscribers
    this->robot_state_sub = this->nh.subscribe<std_msgs::Int8>("/robot_follower_state", 1,
                                                               &ExpMainWindow::robot_state_callback, this);
    this->robot_odom_sub = this->nh.subscribe<nav_msgs::Odometry>("/odom", 1,
                                                                  &ExpMainWindow::robot_odom_callback, this);
    this->cmd_vel_sub = this->nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
                                                                 &ExpMainWindow::cmd_vel_callback, this);

    this->tracking_status_sub = this->nh.subscribe<std_msgs::String>("/tracking/status", 1,
                                                                   &ExpMainWindow::tracking_status_callback, this);
    this->human_pose_sub = this->nh.subscribe<geometry_msgs::Pose2D>("/tracking/human_pos2d", 1,
                                                                     &ExpMainWindow::human_pose_callback, this);
    this->human_vel_sub = this->nh.subscribe<geometry_msgs::Vector3>("/tracking/human_vel2d", 1,
                                                                     &ExpMainWindow::human_vel_callback, this);

    this->imu_acc_sub = this->nh.subscribe<geometry_msgs::Vector3>("/human_input/acc_raw", 1,
                                                                   &ExpMainWindow::imu_acc_callback, this);
    this->imu_gyro_sub = this->nh.subscribe<geometry_msgs::Vector3>("/human_input/gyro_raw", 1,
                                                                   &ExpMainWindow::imu_acc_callback, this);
    this->imu_acc_sub = this->nh.subscribe<geometry_msgs::Vector3>("/human_input/mag_raw", 1,
                                                                   &ExpMainWindow::imu_acc_callback, this);

    this->gesture_rec_sub = this->nh.subscribe<std_msgs::Int8>("/human_input/gesture", 1,
                                                               &ExpMainWindow::gesture_rec_callback, this);

    this->sys_msg_sub = this->nh.subscribe<std_msgs::String>("/sys_message", 1,
                                                             &ExpMainWindow::sys_msg_callback, this);

    // initialize publishers
    this->set_robot_state_pub = this->nh.advertise<std_msgs::Int8>("/state_control/set_state", 1);
    this->haptic_control_pub = this->nh.advertise<ford_project::haptic_msg>("/haptic_control", 1);

    // get parameters
    this->vel_inc_limit_lin = 0.02;
    this->vel_inc_limit_ang = 0.05;

    // initialize other variables
    this->set_state = std_msgs::Int8();
    this->haptic_signal = ford_project::haptic_msg();

    this->cmd_vel = geometry_msgs::Twist();
    this->cmd_vel_goal = geometry_msgs::Twist();
    this->cmd_vel_limits = geometry_msgs::Twist();

    // set update rate
    connect(&m_update_timer, SIGNAL(timeout()), this, SLOT(UpdateGUIInfo()));
    m_update_timer.start(30);

    // setup key bindings

}

// ============================================================================
void ExpMainWindow::UpdateGUIInfo()
{
    // spin ros and check shutdown
    ros::spinOnce();

    if (ros::isShuttingDown()) {
        this->close();
    }
}

// ============================================================================
void ExpMainWindow::robot_state_callback(const std_msgs::Int8::ConstPtr &state_msg)
{
    switch (state_msg->data)
    {
    case 0:
        ui->label_robot_state->setText("Idle");
        ui->label_robot_state->setStyleSheet("QLabel { background-color : yellow; color : black; }");
        break;
    case 1:
        ui->label_robot_state->setText("Following");
        ui->label_robot_state->setStyleSheet("QLabel { background-color : green; color : black; }");
        break;
    case 2:
        ui->label_robot_state->setText("Lost Vision!");
        ui->label_robot_state->setStyleSheet("QLabel { background-color : red; color : black; }");
        break;
    case 3:
        ui->label_robot_state->setText("Robot Stuck!");
        ui->label_robot_state->setStyleSheet("QLabel { background-color : purple; color : black; }");
        break;
    case 4:
        ui->label_robot_state->setText("Teleoperation");
        ui->label_robot_state->setStyleSheet("QLabel { background-color : white; color : black; }");
        break;
    }
}

// ============================================================================
void ExpMainWindow::robot_odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    ui->lcd_vel_current_lin->display(odom_msg->twist.twist.linear.x);
    ui->lcd_vel_current_ang->display(odom_msg->twist.twist.angular.z);
}

// ============================================================================
void ExpMainWindow::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg)
{
    ui->lcd_vel_desire_lin->display(cmd_vel_msg->linear.x);
    ui->lcd_vel_desired_ang->display(cmd_vel_msg->angular.z);
}

// ============================================================================
void ExpMainWindow::tracking_status_callback(const std_msgs::String::ConstPtr &tracking_status_msg)
{
    if (tracking_status_msg->data == "Lost") {
        ui->label_tracking_status->setText("Lost Human");
        ui->label_tracking_status->setStyleSheet("QLabel { background-color : red; color : black; }");
    } else {
        ui->label_tracking_status->setText("Found Human");
        ui->label_tracking_status->setStyleSheet("QLabel { background-color : green; color : black; }");
    }
}

// ============================================================================
void ExpMainWindow::human_pose_callback(const geometry_msgs::Pose2D::ConstPtr &human_pose_msg)
{
    ui->lcd_human_pos_x->display(human_pose_msg->x);
    ui->lcd_human_pos_y->display(human_pose_msg->y);
    ui->lcd_human_pos_theta->display(human_pose_msg->theta);
}

// ============================================================================
void ExpMainWindow::human_vel_callback(const geometry_msgs::Vector3::ConstPtr &human_vel_msg)
{
    ui->lcd_human_vel_x->display(human_vel_msg->x);
    ui->lcd_human_vel_y->display(human_vel_msg->y);
    ui->lcd_human_vel_omg->display(human_vel_msg->z);
}

// ============================================================================
void ExpMainWindow::imu_acc_callback(const geometry_msgs::Vector3::ConstPtr &acc_msg)
{
    ui->lcd_acc_x->display(acc_msg->x);
    ui->lcd_acc_y->display(acc_msg->y);
    ui->lcd_acc_z->display(acc_msg->z);
}

// ============================================================================
void ExpMainWindow::imu_gyro_callback(const geometry_msgs::Vector3::ConstPtr &gyro_msg)
{
    ui->lcd_gyro_x->display(gyro_msg->x);
    ui->lcd_gyro_y->display(gyro_msg->y);
    ui->lcd_gyro_z->display(gyro_msg->z);
}

// ============================================================================
void ExpMainWindow::imu_mag_callback(const geometry_msgs::Vector3::ConstPtr &mag_msg)
{
    ui->lcd_mag_x->display(mag_msg->x);
    ui->lcd_mag_y->display(mag_msg->y);
    ui->lcd_mag_z->display(mag_msg->z);
}

// ============================================================================
void ExpMainWindow::imu_tilt_callback(const geometry_msgs::Vector3::ConstPtr &tilt_msg)
{
    ui->lcd_tilt_roll->display(tilt_msg->x);
    ui->lcd_tilt_pitch->display(tilt_msg->y);
    ui->lcd_tilt_yaw->display(tilt_msg->z);
}

// ============================================================================
void ExpMainWindow::gesture_rec_callback(const std_msgs::Int8::ConstPtr &gesture_msg)
{
    switch (gesture_msg->data)
    {
    case 0:
        ui->lineEdit_gesture_rec->setText("Recognized Gesgure: Forward");
        break;
    case 1:
        ui->lineEdit_gesture_rec->setText("Recognized Gesgure: Backward");
        break;
    case 2:
        ui->lineEdit_gesture_rec->setText("Recognized Gesgure: Turn Left");
        break;
    case 3:
        ui->lineEdit_gesture_rec->setText("Recognized Gesgure: Turn Right");
        break;
    case 4:
        ui->lineEdit_gesture_rec->setText("Recognized Gesgure: Stop Following");
        break;
    case 5:
        ui->lineEdit_gesture_rec->setText("Recognized Gesgure: Start Following");
        break;
    }
}

// ============================================================================
void ExpMainWindow::sys_msg_callback(const std_msgs::String::ConstPtr &sys_msg)
{
    ui->browser_sys_message->append(QString::fromStdString(sys_msg->data));
}

// ============================================================================
void ExpMainWindow::on_combo_set_state_currentIndexChanged(int index)
{
    switch (index)
    {
    case 0:
        this->set_state.data = 0;
        break;
    case 1:
        this->set_state.data = 1;
        break;
    case 2:
        this->set_state.data = 2;
        break;
    }
}

// ============================================================================
void ExpMainWindow::on_button_set_state_clicked()
{
    this->set_robot_state_pub.publish(this->set_state);
}

// ============================================================================
void ExpMainWindow::on_spinBox_vel_lin_valueChanged(double arg1)
{
    this->cmd_vel_limits.linear.x = arg1;
}

// ============================================================================
void ExpMainWindow::on_spinBox_vel_ang_valueChanged(double arg1)
{
    this->cmd_vel_limits.angular.z = arg1;
}

// ============================================================================
void ExpMainWindow::on_button_tele_forward_pressed()
{
    std::cout << "pressed!!!" << std::endl;
}

// ============================================================================
void ExpMainWindow::on_combo_haptic_dir_currentIndexChanged(int index)
{
    this->haptic_signal.direction = index;
}

// ============================================================================
void ExpMainWindow::on_spinBox_haptic_rep_valueChanged(int arg1)
{
    this->haptic_signal.repetition = arg1;
}

// ============================================================================
void ExpMainWindow::on_spinBox_haptic_on_valueChanged(double arg1)
{
    this->haptic_signal.period_render = arg1;
}

// ============================================================================
void ExpMainWindow::on_spinBox_haptic_off_valueChanged(double arg1)
{
    this->haptic_signal.period_pause = arg1;
}

// ============================================================================
void ExpMainWindow::on_button_send_haptic_clicked()
{
    // check if the signal is valid

    // publish the signal
    this->haptic_control_pub.publish(this->haptic_signal);
}

// ============================================================================
void ExpMainWindow::on_combo_haptic_mag_currentIndexChanged(int index)
{
    switch(index) {
    case 0:
        this->haptic_signal.amplitude = 0.8;
        break;
    case 1:
        this->haptic_signal.amplitude = 1.0;
        break;
    case 2:
        this->haptic_signal.amplitude = 1.2;
        break;
    }
}