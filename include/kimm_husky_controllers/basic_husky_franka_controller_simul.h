// Controller
#include "kimm_husky_controllers/husky_franka_hqp.h"

//Mujoco MSG Header
#include "mujoco_ros_msgs/JointSet.h"
#include "mujoco_ros_msgs/SensorState.h"

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "visualization_msgs/Marker.h"

// Tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

ros::Publisher mujoco_command_pub_;
ros::Publisher robot_command_pub_;
ros::Publisher ee_state_pub_, base_state_pub_, husky_odom_pub_;

mujoco_ros_msgs::JointSet robot_command_msg_;
geometry_msgs::Transform ee_state_msg_;
sensor_msgs::JointState base_state_msg_;

double mujoco_time_, time_;
bool isgrasp_;
Eigen::VectorXd franka_qacc_, husky_qvel_, robot_nle_, robot_g_, franka_torque_;
Eigen::MatrixXd robot_mass_, robot_J_;
string group_name;

RobotController::HuskyFrankaWrapper * ctrl_;
Mob mob_;
State state_;
tf::TransformBroadcaster* br_;

void simCommandCallback(const std_msgs::StringConstPtr &msg);
void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void ctrltypeCallback(const std_msgs::Int16ConstPtr &msg);

void setRobotCommand();
void setGripperCommand();
void getEEState();
void getBaseState();
void odomPublish();

void InitMob(){
    mob_.torque_d_prev_.setZero(7);
    mob_.d_torque_.setZero(7);
    mob_.p_k_prev_.setZero(7);         
}
void UpdateMob(){
    franka_torque_.head(7) -= mob_.d_torque_;

    mob_.gamma_ = exp(-15. * 0.001);
    mob_.beta_ = (1- mob_.gamma_) / (mob_.gamma_ * 0.001);

    mob_.coriolis_ = robot_nle_ - robot_g_;
    mob_.p_k_ = robot_mass_ * state_.v_;
    mob_.alpha_k_ = mob_.beta_*mob_.p_k_ + franka_torque_.head(7) + mob_.coriolis_ - robot_g_;

    mob_.torque_d_ = (mob_.gamma_-1.) * mob_.alpha_k_ + mob_.beta_ * (mob_.p_k_ - mob_.gamma_ * mob_.p_k_prev_) + mob_.gamma_ * mob_.torque_d_prev_;
    mob_.mass_inv_ = robot_mass_.inverse();
    mob_.lambda_inv_ = robot_J_ * mob_.mass_inv_ * robot_J_.transpose();
    mob_.lambda_ = mob_.lambda_inv_.inverse();
    
    Eigen::MatrixXd J_trans = robot_J_.transpose();
    Eigen::MatrixXd J_trans_inv = J_trans.completeOrthogonalDecomposition().pseudoInverse();

    mob_.d_force_ = (mob_.mass_inv_ * J_trans *mob_.lambda_).transpose() * mob_.torque_d_;

    mob_.d_torque_ = -J_trans * mob_.d_force_;
    mob_.torque_d_prev_ = mob_.torque_d_;
    mob_.p_k_prev_ = mob_.p_k_;

}
void BaseStatusCallback(sensor_msgs::JointState & msg, const pinocchio::SE3 & H_base);
void EEStatusCallback(geometry_msgs::Transform& msg, const pinocchio::SE3 & H_ee);

void keyboard_event();
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
};

