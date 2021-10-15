#include "kimm_husky_controllers/basic_husky_franka_controller_simul.h"

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace RobotController;

int main(int argc, char **argv)
{   
    //Ros setting
    ros::init(argc, argv, "husky_franka_controller");
    ros::NodeHandle n_node;

    ros::Rate loop_rate(1000);

    // Robot Wapper
    
    n_node.getParam("/robot_group", group_name);    
    ctrl_ = new RobotController::HuskyFrankaWrapper(group_name, true, n_node);
    ctrl_->initialize();

    // mujoco sub
    ros::Subscriber jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));    
    ros::Subscriber mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber ctrl_type_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/ctrl_type", 1, &ctrltypeCallback, ros::TransportHints().tcpNoDelay(true));

    // mujoco pub
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);

    // robot pub
    base_state_pub_ = n_node.advertise<sensor_msgs::JointState>("mujoco_ros/mujoco_ros_interface/base_state", 5);
    ee_state_pub_ = n_node.advertise<geometry_msgs::Transform>("mujoco_ros/mujoco_ros_interface/ee_state", 5);
    br_ = new tf::TransformBroadcaster();

    
    husky_odom_pub_ = n_node.advertise<visualization_msgs::Marker>("/husky_odom", 1);

    // msg 
    robot_command_msg_.torque.resize(13); // gripper(2) + wheels(4) + robot (7)
    base_state_msg_.position.resize(3);
    base_state_msg_.velocity.resize(3);
    ee_state_msg_ = geometry_msgs::Transform();

    isgrasp_ = false;
    // InitMob();
    tf::TransformListener listener;
    
    while (ros::ok()){
        
        tf::StampedTransform transform2;
        tf::Vector3 origin;
        try{
            listener.lookupTransform("/map", "/" + group_name + "_carto_base_link" , ros::Time(0), transform2);
        }
        catch (tf::TransformException ex){

        }
        origin = transform2.getOrigin();
        tf::Quaternion q2;
        q2 = transform2.getRotation();
        SE3 odom;
        odom.translation()(0) = transform2.getOrigin().x();
        odom.translation()(1) = origin.getY();
        odom.translation()(2) = origin.getZ();
        Eigen::Quaterniond quat;
        quat.x() = q2.x();
        quat.y() = q2.y();
        quat.z() = q2.z();
        quat.w() = q2.w();
        quat.normalize();
        odom.rotation() = quat.toRotationMatrix();
       
        keyboard_event();

        // ctrl computation
        ctrl_->compute(time_);
        
        // get output
        ctrl_->mass(robot_mass_);
        ctrl_->nle(robot_nle_);
        ctrl_->g(robot_g_);
        ctrl_->J(robot_J_);
        ctrl_->state(state_);
        
        ctrl_->franka_output(franka_qacc_); // this is only for simulation mode
        ctrl_->husky_output(husky_qvel_);

        franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;
        
        // if (ctrl_->ctrltype() != 0)
        //     UpdateMob();
        // else
        //     InitMob();

        setGripperCommand();
        setRobotCommand();
        robot_command_pub_.publish(robot_command_msg_);
       
        getBaseState();
        getEEState();
        
        ros::spinOnce();
        loop_rate.sleep();
    }//while

    return 0;
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    
    sensor_msgs::JointState msg_tmp;
    msg_tmp = *msg;
    // if (group_name == "ns0")
    //     msg_tmp.position[1] = msg_tmp.position[1] + 2.0;

    ctrl_->franka_update(msg_tmp);
    ctrl_->husky_update(msg_tmp);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg_tmp.position[0], msg_tmp.position[1], msg_tmp.position[2]) );
    tf::Quaternion q(msg_tmp.position[4], msg_tmp.position[5], msg_tmp.position[6], msg_tmp.position[3]);
    //tf::Quaternion q(0, 0, 0, 1);
    transform.setRotation(q);

    br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "husky_odom", group_name + "_rviz_base_link"));
    
    
}

void ctrltypeCallback(const std_msgs::Int16ConstPtr &msg){
    ROS_WARN("%d", msg->data);
    
    if (msg->data != 899){
        int data = msg->data;
        ctrl_->ctrl_update(data);
    }
    else{
        if (isgrasp_)
            isgrasp_=false;
        else
            isgrasp_=true;
    }
}

void setRobotCommand(){
    robot_command_msg_.MODE = 1;
    robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;
    
    robot_command_msg_.torque[0] = husky_qvel_(0);
    robot_command_msg_.torque[2] = husky_qvel_(0);
    robot_command_msg_.torque[1] = husky_qvel_(1);
    robot_command_msg_.torque[3] = husky_qvel_(1);

    for (int i=0; i<7; i++)
        robot_command_msg_.torque[i+4] = franka_torque_(i);    
}

void setGripperCommand(){
    if (isgrasp_){
        robot_command_msg_.torque[11] = -200.0;
        robot_command_msg_.torque[12] = -200.0;
    }
    else{
        robot_command_msg_.torque[11] = 100.0;
        robot_command_msg_.torque[12] = 100.0;
    }
}

void getEEState(){
    Vector3d pos;
    Quaterniond q;
    ctrl_->ee_state(pos, q);

    ee_state_msg_.translation.x = pos(0);
    ee_state_msg_.translation.y = pos(1);
    ee_state_msg_.translation.z = pos(2);

    ee_state_msg_.rotation.x = q.x();
    ee_state_msg_.rotation.y = q.y();
    ee_state_msg_.rotation.z = q.z();
    ee_state_msg_.rotation.w = q.w();
    ee_state_pub_.publish(ee_state_msg_);
}

void getBaseState(){
    Vector3d pos;
    ctrl_->base_state(pos);

    for (int i=0; i<3; i++)
        base_state_msg_.position[i] = pos(i);

    base_state_pub_.publish(base_state_msg_);
}

void odomPublish(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "husky_odom";
    marker.header.stamp = ros::Time();
    marker.ns = "husky_odom";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    husky_odom_pub_.publish( marker );
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'g': //gravity
                msg = 0;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Gravity mode" << endl;
                cout << " " << endl;
                break;
            case 'h': //home
                msg = 1;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Initialization" << endl;
                cout << " " << endl;
                break;
            case 'a': //home
                msg = 2;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break;    
            case 's': //home
                msg = 3;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break;    
            case 'd': //home
                msg = 4;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break;  
            case 'f': //home
                msg = 5;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break;   
            case 'x': //home
                msg = 6;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break; 
            case 'z': //grasp
                if (isgrasp_){
                    cout << "Release hand" << endl;
                    isgrasp_ = false;
                }
                else{
                    cout << "Grasp object" << endl;
                    isgrasp_ = true; 
                }
                break;
        }
    }
}

