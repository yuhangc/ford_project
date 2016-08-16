#include "gesture_DTW.h"
#include <iostream>
#include <fstream>
#include <sstream>

// ============================================================================
// constructor
GestureDTW::GestureDTW()
{
    // initialize subscriber and publishers
    this->acc_data_sub = this->nh.subscribe<geometry_msgs::Vector3>("/human_input/acc_raw",
                                                                    1, &GestureDTW::acc_data_callback, this);
    this->gesture_pub = this->nh.advertise<std_msgs::Int8>("/human_input/gesture", 1);

    // initialize the "constants"
    ros::param::param<int>("~num_gestures", this->num_gestures, 6);
    ros::param::param<int>("~max_window_size", this->max_window_size, 45);
    ros::param::param<int>("~dtw_window", this->dtw_window, 45);

    this->time_debounce = 0.5;
    this->time_gesture_start = 0;

    double vals[] = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
    this->rej_corr.assign(vals, vals+this->num_gestures);

    // flags
    this->flag_gesture = false;

    //! initialize state
    this->state = State_Detection_Start;

    // load file
    std::string file_path;
    ros::param::param<std::string>("~gesture_data_path", file_path, "gesture_data");
    this->load_data(file_path);
}

// ============================================================================
// distructor
GestureDTW::~GestureDTW()
{
}

// ============================================================================
void GestureDTW::acc_data_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    this->acc_raw(0) = msg->x;
    this->acc_raw(1) = msg->y;
    this->acc_raw(2) = msg->z;

    // filter data and add to history
    this->add_filtered_acc();

    // state machine for gesture detection
    switch (this->state)
    {
    case State_Detection_Idle:
        break;

    case State_Detection_Start:
        // perform gesture detection
        this->gesture_matching();

        // switch to debounce once gesture detected
        if (this->flag_gesture) {
            // publish gesture
            this->gesture_pub.publish(this->gesture_id);

            this->flag_gesture = false;
            this->state = State_Detection_Debounce;
            this->time_gesture_start = ros::Time::now().toSec();
            // std::cout << "gesture " << this->gesture_id << std::endl;
        }
        break;

    case State_Detection_Debounce:
        // check time
        double t_debounce = ros::Time::now().toSec() - this->time_gesture_start;
        if (t_debounce >= this->time_debounce) {
            // go back to detection state
            this->state = State_Detection_Start;
        }
        break;
    }
}

// ============================================================================
void GestureDTW::load_data(std::string file_path)
{
    // open and read the threshold file
    std::ifstream data_file;
    std::stringstream full_path;
    full_path << file_path << "/rejection_threshold.txt";
    data_file.open(full_path.str().c_str());

    double new_thresh;
    for (int g = 0; g < this->num_gestures; g++)
    {
        data_file >> new_thresh;
        this->rej_thresh.push_back(new_thresh);
    }
    data_file.close();

    // open and read the gesture file
    for (int g = 0; g < this->num_gestures; g++)
    {
        full_path.str("");
        full_path << file_path << "/template" << g << ".txt";
        data_file.open(full_path.str().c_str());

        // get data into a vector first
        std::vector<std::string> data;
        std::string line;
        while (std::getline(data_file, line))
        {
            data.push_back(line);
        }

        // convert to matrix
        Eigen::MatrixXd temp(data.size(), 3);
        for (int i = 0; i < data.size(); i++)
        {
            std::stringstream ss(data[i]);
            ss >> temp(i, 0);
            ss.ignore(1);
            ss >> temp(i, 1);
            ss.ignore(1);
            ss >> temp(i, 2);
            std::cout << temp(i, 0) << " " << temp(i, 1) << std::endl;
        }

        // add gesture template
        this->gesture_temps.push_back(temp);
        std::cout << "template " << g << " loaded!" << std::endl;

        // close the file
        data_file.close();
    }
}

// ============================================================================
void GestureDTW::add_filtered_acc()
{
    if (this->acc_filtered.empty()) {
        this->acc_filtered.push_back(this->acc_raw);
    } else {
        Eigen::Vector3d acc_new = 0.5 * this->acc_raw + 0.5 * this->acc_filtered.back();
        this->acc_filtered.push_back(acc_new);

        // maintain the size
        if (this->acc_filtered.size() > this->max_window_size) {
            this->acc_filtered.pop_front();
        }
    }
}

// ============================================================================
void GestureDTW::gesture_matching()
{
    double gesture_score = 0;
    double min_score = 1e6;

    // only do gesture detection after having enough data
    if (this->acc_filtered.size() < this->max_window_size) {
        return;
    }

    // perform DTW on each gesture template
    for (int g = 0; g < this->num_gestures; g++) {
        Eigen::MatrixXd temp = this->gesture_temps[g];
        int temp_len = temp.rows();

        // initialize the cost array
        double w[this->max_window_size+1][temp_len+1];
        for (int i = 0; i < this->max_window_size+1; i++) {
            for (int j = 0; j < temp_len+1; j++) {
                w[i][j] = 1e6;
            }
        }
        w[0][0] = 0;

        // DTW
        for (int i = 0; i < this->max_window_size; i++) {
            w[i+1][0] = w[i][0] + dist(this->acc_filtered[i], temp.row(0));
        }

        for (int j = 0; j < temp_len; j++) {
            w[0][j+1] = w[0][j] + dist(this->acc_filtered[0], temp.row(j));
        }

        for (int i = 0; i < this->max_window_size; i++) {
            int j_start = std::max(0, i - this->dtw_window);
            int j_end = std::min(temp_len, i + this->dtw_window);
            for (int j = j_start; j < j_end; j++) {
                w[i+1][j+1] = std::min(w[i][j+1], std::min(w[i+1][j], w[i][j])) +
                        dist(this->acc_filtered[i], temp.row(j));
            }
        }

        // check if greater than threshold
        gesture_score = w[this->max_window_size][temp_len] / temp_len;
        if (gesture_score < this->rej_thresh[g] * this->rej_corr[g]){
            this->flag_gesture = true;
            if (gesture_score < min_score) {
                min_score = gesture_score;
                this->gesture_id.data = g;
            }
        }
//        std::cout << gesture_score << " ";
    }
//    std::cout << std::endl;
}

// ============================================================================
double GestureDTW::dist(Eigen::VectorXd v1, Eigen::VectorXd v2)
{
    Eigen::VectorXd v_diff = v1 - v2;
    return v_diff.norm();
}

// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gesture_recognition");
    GestureDTW gesture_rec = GestureDTW();

    ros::Rate loop_rate(50);
    while (!ros::isShuttingDown()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
