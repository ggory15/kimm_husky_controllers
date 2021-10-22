#include "kimm_husky_controllers/husky_franka_hqp.h"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace kimmhqp;
using namespace kimmhqp::trajectory;
using namespace kimmhqp::math;
using namespace kimmhqp::tasks;
using namespace kimmhqp::solver;
using namespace kimmhqp::robot;
using namespace kimmhqp::contacts;

namespace RobotController{
    HuskyFrankaWrapper::HuskyFrankaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node)
    : robot_node_(robot_node), issimulation_(issimulation), n_node_(node)
    {
        time_ = 0.;
        mode_change_ = false;
        ctrl_mode_ = 0;
        node_index_ = 0;
        cnt_ = 0;
    }

    void HuskyFrankaWrapper::initialize(){
        // Robot
        string model_path, urdf_name;
        n_node_.getParam("/" + robot_node_ +"/robot_urdf_path", model_path);    
        n_node_.getParam("/" + robot_node_ +"/robot_urdf", urdf_name);  

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;
        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, true, false);
        model_ = robot_->model();
        na_ = robot_->na();
        nv_ = robot_->nv();
        nq_ = robot_->nq();

        // State
        state_.q_.setZero(nq_);
        state_.v_.setZero(nv_);
        state_.dv_.setZero(nv_);
        state_.torque_.setZero(na_);

        // tsid
        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q_, state_.v_);
        data_ = tsid_->data();

        // tasks
        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
        VectorXd posture_gain(na_ -2);
        if (!issimulation_)
        	posture_gain << 200., 200., 200., 200., 200., 200., 200.;
        else
        	posture_gain << 4000., 4000., 4000., 4000., 4000., 4000., 4000.;
        	
        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());

        Vector3d ee_offset(0.0, 0, 0.0);
        VectorXd ee_gain(6);
        ee_gain << 100., 100., 100., 400., 400., 400.;
        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7", ee_offset);
        eeTask_->Kp(ee_gain*Vector::Ones(6));
        eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());
        
        torqueBoundsTask_ = std::make_shared<TaskJointBounds>("task-torque-bounds", *robot_);
        Vector dq_max = 500000.0*Vector::Ones(na_);    
        dq_max(0) = 500.;
        dq_max(1) = 500.;
        Vector dq_min = -dq_max;
        torqueBoundsTask_->setJointBounds(dq_min, dq_max);

        mobileTask_ = std::make_shared<TaskMobileEquality>("task-mobile", *robot_, true);
        mobileTask_->Kp(50.0*Vector3d::Ones());    
        mobileTask_->Kd(2.5*mobileTask_->Kp().cwiseSqrt());

        mobileTask2_ = std::make_shared<TaskMobileEquality>("task-mobile2", *robot_, false);
        mobileTask2_->Kp(50.0*Vector3d::Ones());
        mobileTask2_->Kd(2.5*mobileTask2_->Kp().cwiseSqrt());

        // trajecotries
        sampleEE_.resize(12, 6);
        samplePosture_.resize(na_- 2);

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");
        
        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);

        Vector3d Maxvel_base = Vector3d::Ones()*1.0;
        Vector3d Maxacc_base = Vector3d::Ones()*1.0;
        trajMobile_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_mobile");
        trajMobile_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_mobile_constant");
        trajMobile_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_mobile_timeopt", Maxvel_base, Maxacc_base);

        // solver
        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_QPOASES, "qpoases");           

        // service
        joint_action_subs_ = n_node_.subscribe("/" + robot_node_ + "_gui/kimm_joint_planner_ros_interface_server/joint_action", 1, &RobotController::HuskyFrankaWrapper::jointActionCallback, this);
        se3_action_subs_ = n_node_.subscribe("/" + robot_node_ + "_gui/kimm_se3_planner_ros_interface_server/se3_action", 1, &RobotController::HuskyFrankaWrapper::se3ActionCallback, this);
        mobile_action_subs_ = n_node_.subscribe("/" + robot_node_ + "_gui/kimm_path_planner_ros_interface_server/mobile_action", 1, &RobotController::HuskyFrankaWrapper::mobileActionCallback, this);
        //mobile_action_client_ = n_node_.serviceClient<kimm_path_planner_ros_interface::action_mobile_path>("/" + robot_node_ + "_gui/kimm_path_planner_ros_interface_server/action_mobile_path");
        //joint_action_client_ = n_node_.serviceClient<kimm_joint_planner_ros_interface::action_joint_path>("/" + robot_node_ + "_gui/kimm_joint_planner_ros_interface_server/action_joint_path");
        // se3_action_client_ = n_node_.serviceClient<kimm_se3_planner_ros_interface::action_se3_path>("/" + robot_node_ + "_gui/kimm_se3_planner_ros_interface_server/action_se3_path");    
        reset_control_ = true;
        planner_res_ = false;
    }
    
    void HuskyFrankaWrapper::franka_update(const sensor_msgs::JointState& msg){
        assert(issimulation_);
        for (int i=0; i< 7; i++){ 
            state_.q_(i+5) = msg.position[i+11];
            state_.v_(i+5) = msg.velocity[i+10];
        }
    }
    void HuskyFrankaWrapper::franka_update(const Vector7d& q, const Vector7d& qdot){
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;
    }
    void HuskyFrankaWrapper::husky_update(const sensor_msgs::JointState& msg){
        assert(issimulation_);
        for (int i=0; i<2; i++){
            state_.q_(i) = msg.position[i];
            state_.v_(i) = msg.velocity[i];
        }
    
        double theta = atan2(2.* (msg.position[5] * msg.position[4] + msg.position[6] * msg.position[3]), 1- 2.*(pow( msg.position[6], 2) + pow(msg.position[5], 2)));
        state_.q_(2) = theta;       
        state_.v_(2) = msg.velocity[5];
        
        // only for front wheel (not used)
        state_.q_(3) = msg.position[7];
        state_.q_(4) = msg.position[8];
        state_.v_(3) = msg.velocity[6];
        state_.v_(4) = msg.velocity[7];
    }
    void HuskyFrankaWrapper::husky_update(const Vector3d& base_pos, const Vector3d& base_vel, const Vector2d& wheel_pos, const Vector2d& wheel_vel){
        assert(!issimulation_);
        for (int i=0; i<3; i++){
            state_.q_(i) = base_pos(i);
            state_.v_(i) = base_vel(i);
        }
        for (int i=0; i<2; i++){
            state_.q_(i+3) = wheel_pos(i);
            state_.v_(i+3) = wheel_vel(i);
        }
    }

    void HuskyFrankaWrapper::ctrl_update(const int& msg){
        ctrl_mode_ = msg;
        ROS_INFO("[ctrltypeCallback] %d", ctrl_mode_);
        mode_change_ = true;
    }

    void HuskyFrankaWrapper::compute(const double& time){
        time_ = time;

        robot_->computeAllTerms(data_, state_.q_, state_.v_);

        if (ctrl_mode_ == 0){ // gravity mode
            state_.torque_.setZero();
        }
        if (ctrl_mode_ == 1){
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                
                tsid_->addMotionTask(*postureTask_, 1e-5, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0;//M_PI /4.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                q_ref_(6) = -M_PI/ 4.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(5.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);      
                            
                reset_control_ = false;
                mode_change_ = false;                
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);
           
            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       

            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
            
        }
        if (ctrl_mode_ == 2){
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-2, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
              //  tsid_->addMotionTask(*mobileTask_, 1.0, 1);

                //traj
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2)); 

                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(5.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                H_ee_ref_.translation()(0) = 2.5;
                if (robot_node_ == "ns0")  
                    H_ee_ref_.translation()(1) = -0.6;
                else    
                    H_ee_ref_.translation()(1) = +0.6;
                H_ee_ref_.translation()(2) = 0.5;  
                
                H_ee_ref_.rotation().col(0) << cos(M_PI/4.0), sin(M_PI/4.0), 0;
                H_ee_ref_.rotation().col(1) << cos(M_PI/4.0), -sin(M_PI/4.0), 0;
                H_ee_ref_.rotation().col(2) << 0, 0, -1;

                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                // H_mobile_ref_.rotation().col(0) << -1, 0, 0;
                // H_mobile_ref_.rotation().col(1) << 0, -1, 0;
                // H_mobile_ref_.rotation().col(2) << 0, 0, 1;
                
                reset_control_ = false;
                mode_change_ = false;
            }
            
            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);     

            trajEE_Cubic_->setCurrentTime(time_);            
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));        

        }      
        if (ctrl_mode_ == 3){
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0;//M_PI /4.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                q_ref_(6) = -M_PI/ 4.0;

                //traj
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_); 

                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(5.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                trajEE_Cubic_->setInitSample(H_ee_ref_);     
                H_ee_ref_.rotation().col(0) << cos(M_PI/4.0), sin(M_PI/4.0), 0;
                H_ee_ref_.rotation().col(1) << cos(M_PI/4.0), -sin(M_PI/4.0), 0;
                H_ee_ref_.rotation().col(2) << 0, 0, -1;
                H_ee_ref_.translation()(2) = -0.03;
                // H_ee_ref_.translation()(2) -= 0.025 * cnt_;
                                       
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                // H_mobile_ref_.rotation().col(0) << -1, 0, 0;
                // H_mobile_ref_.rotation().col(1) << 0, -1, 0;
                // H_mobile_ref_.rotation().col(2) << 0, 0, 1;
                
                reset_control_ = false;
                mode_change_ = false;
            }
            
            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);     

            trajEE_Cubic_->setCurrentTime(time_);            
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));        
        }         
        if (ctrl_mode_ == 4){
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0;//M_PI /4.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = - M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                q_ref_(6) = -M_PI/ 4.0;

                //traj
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_); 

                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(5.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                trajEE_Cubic_->setInitSample(H_ee_ref_);     
                H_ee_ref_.translation()(2) = 0.5;                   
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                // H_mobile_ref_.rotation().col(0) << -1, 0, 0;
                // H_mobile_ref_.rotation().col(1) << 0, -1, 0;
                // H_mobile_ref_.rotation().col(2) << 0, 0, 1;
                
                reset_control_ = false;
                mode_change_ = false;
            }
            
            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);     

            trajEE_Cubic_->setCurrentTime(time_);            
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));        

        }     
        if (ctrl_mode_ == 5){
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask_, 1.0, 0);

                //traj
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2)); 

                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(5.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                trajEE_Cubic_->setInitSample(H_ee_ref_);     
                H_ee_ref_.translation()(0) -= 1.0;                   
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(5.0);
                trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                H_mobile_ref_.rotation().col(0) << 1, 0, 0;
                H_mobile_ref_.rotation().col(1) << 0, 1, 0;
                H_mobile_ref_.rotation().col(2) << 0, 0, 1;
                trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
                
                reset_control_ = false;
                mode_change_ = false;
            }
            
            // husky
            trajMobile_Cubic_->setCurrentTime(time_);
            sampleMobile_ = trajMobile_Cubic_->computeNext();
            mobileTask_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);     

            trajEE_Cubic_->setCurrentTime(time_);            
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));        
         //   state_.torque_(0) = 500;
         //   state_.torque_(1) = 500;
        }       
        if (ctrl_mode_ == 6){
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask_, 1.0, 0);

                //traj
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2)); 

                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(5.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                trajEE_Cubic_->setInitSample(H_ee_ref_);     
                H_ee_ref_.translation()(0) += 1.0;                   
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                // H_mobile_ref_.rotation().col(0) << -1, 0, 0;
                // H_mobile_ref_.rotation().col(1) << 0, -1, 0;
                // H_mobile_ref_.rotation().col(2) << 0, 0, 1;
                
                reset_control_ = false;
                mode_change_ = false;
            }
            
            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);     

            trajEE_Cubic_->setCurrentTime(time_);            
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));              
        }

        ///////////////////////// Predefined CTRL for GUI //////////////////////////////////////////////////
        if (ctrl_mode_ == 887){
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                
                tsid_->addMotionTask(*postureTask_, 1e-5, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_ = state_.q_.tail(7);
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(5.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);      
                            
                reset_control_ = false;
                mode_change_ = false;                
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);
           
            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       

            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
            
        }
        if (ctrl_mode_ == 888){
            if (mode_change_){
                //remove
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                //add
                tsid_->addMotionTask(*postureTask_, 1e-2, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*mobileTask_, 1.0, 1);

                //traj
                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2)); 

                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(5.0);
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                // H_mobile_ref_.rotation().col(0) << -1, 0, 0;
                // H_mobile_ref_.rotation().col(1) << 0, -1, 0;
                // H_mobile_ref_.rotation().col(2) << 0, 0, 1;
                
                reset_control_ = false;
                mode_change_ = false;
            }
            
            // husky
            trajMobile_Constant_->setReference(H_mobile_ref_);
            sampleMobile_ = trajMobile_Constant_->computeNext();
            mobileTask_->setReference(sampleMobile_);

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);     

            trajEE_Cubic_->setCurrentTime(time_);            
            sampleEE_ = trajEE_Cubic_->computeNext();
            sampleEE_.pos.head(2) = robot_->position(data_, robot_->model().getJointId("panda_joint7")).translation().head(2);
            
            eeTask_->setReference(sampleEE_);        

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));        
        }
        if (ctrl_mode_ == 900){ // joint ctrl
            if (mode_change_){
                mode_change_ = false;
                reset_control_ = false;

                if (!joint_action_.is_succeed_){
                    tsid_->removeTask("task-mobile");
                    tsid_->removeTask("task-mobile2");
                    tsid_->removeTask("task-se3");
                    tsid_->removeTask("task-posture");
                    tsid_->removeTask("task-torque-bounds");

                    tsid_->addMotionTask(*postureTask_, 1e-5, 1);
                    tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                    if (joint_action_.type_== 1 || joint_action_.type_ == 2){
                        trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                        trajPosture_Cubic_->setDuration(joint_action_.duration_);
                        trajPosture_Cubic_->setStartTime(time_);
                        trajPosture_Cubic_->setGoalSample(joint_action_.q_target_); 
                    }
                    stime_ = time_;
                    postureTask_->Kp(joint_action_.kp_);
                    postureTask_->Kd(joint_action_.kd_);
                   
                    ROS_WARN("%f", joint_action_.duration_);
                }
            }           
            if (!joint_action_.is_succeed_){
                if (joint_action_.type_ == 0){
                    samplePosture_.pos.setZero(na_-2);
                    samplePosture_.vel.setZero(na_-2);
                    samplePosture_.acc.setZero(na_-2);
                    samplePosture_.pos = joint_action_.q_target_;                    
                } 
                else{
                    trajPosture_Cubic_->setCurrentTime(time_);
                    samplePosture_ = trajPosture_Cubic_->computeNext();
                    postureTask_->setReference(samplePosture_);     
                }
                const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
                const HQPOutput & sol = solver_->solve(HQPData);

                state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
                state_.torque_.head(2).setZero();
/*
                if (time_ > stime_ + joint_action_.duration_ + 3.0){
                    //joint_action_.is_succeed_ = true;
		    state_.torque_.setZero();
		}
*/
            }
            else{
                state_.torque_.setZero();
            }
        }
        if (ctrl_mode_ == 901){ // base ctrl
            if (mode_change_){
                if (!mobile_action_.is_succeed_){
                    node_num_ = mobile_action_.path_.size() -1;
                    tsid_->removeTask("task-posture");
                    tsid_->removeTask("task-torque-bounds");
                    tsid_->removeTask("task-mobile");
                    tsid_->removeTask("task-mobile2");
                    tsid_->removeTask("task-se3");

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                    double theta = atan2(-H_mobile_ref_.rotation()(0, 1), H_mobile_ref_.rotation()(0,0));
                    
                    tsid_->addMotionTask(*mobileTask_, 0.1, 1);
                    tsid_->addMotionTask(*mobileTask2_, 1, 1);
                    tsid_->addMotionTask(*postureTask_, 1e-5, 1);
                    tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(0.001);
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
                    trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));

                    mode_change_ = false;
                    update_weight_ = false;
                    stime_ = time_;

                    prev_node_ = -1;
                    node_index_ = 0;

                    trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                    trajPosture_Cubic_->setDuration(0.1);
                    trajPosture_Cubic_->setStartTime(time_);
                    trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));    
                    reset_control_ = false;
                }
                else{
                    mode_change_ = false;
                }
            }           
            if (!mobile_action_.is_succeed_){
                Vector3d goal_path; 
                if (node_num_ != -1){
                    goal_path << mobile_action_.path_[node_index_].x, mobile_action_.path_[node_index_].y, mobile_action_.path_[node_index_].theta; 
                    H_mobile_ref_.translation().head(2) = goal_path.head(2);

                    H_mobile_ref_.rotation().setIdentity();
                    H_mobile_ref_.rotation()(0, 0) = cos(goal_path(2));
                    H_mobile_ref_.rotation()(0, 1) = -sin(goal_path(2));
                    H_mobile_ref_.rotation()(1, 0) = sin(goal_path(2));
                    H_mobile_ref_.rotation()(1, 1) = cos(goal_path(2));
                }            
            
                if (node_index_ < node_num_){
                    if (prev_node_ != node_index_){                   
                        trajMobile_Cubic_->setStartTime(time_);
                        trajMobile_Cubic_->setDuration(0.001);
                        trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
                        trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));

                        prev_node_ = node_index_;
                        stime_ = time_;
                    }

                    if (stime_ + 0.1 <= time_)
                        node_index_ += 1;              
                }
                else{
                    if (!update_weight_){
                        tsid_->updateTaskWeight("task-mobile", 1.);
                        tsid_->updateTaskWeight("task-mobile2", 0.01);

                        trajMobile_Cubic_->setStartTime(time_);
                        trajMobile_Cubic_->setDuration(0.1);
                        trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
                        trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));

                        update_weight_ = true;
                    }         
                }

                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();

                mobileTask2_-> setReference(sampleMobile_);
                mobileTask_ -> setReference(sampleMobile_);
            
                trajPosture_Cubic_->setCurrentTime(time_);
                samplePosture_ = trajPosture_Cubic_->computeNext();
                postureTask_->setReference(samplePosture_);

                const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
                const HQPOutput & sol = solver_->solve(HQPData);

                state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
                state_.torque_.head(2) = tsid_->getAccelerations(solver_->solve(HQPData)).head(2);
                
                if (time_ > stime_ + node_num_ * 0.1 + 7.0){
                    state_.torque_.head(2).setZero();
                    //mobile_action_.is_succeed_ = true;
                }
            }
            else{
                state_.torque_.setZero();
            }
        }
        if (ctrl_mode_ == 902){ //base forward
            if (mode_change_){
                stime_ = time_;
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");

                tsid_->addMotionTask(*postureTask_, 1e-5, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));  

                mode_change_ = false;
                reset_control_ = false;
            }
            if (stime_ + 5.0 > time_){
                state_.torque_(0) = 50.;
                state_.torque_(1) = 50.;
            }
            else{
                state_.torque_(0) = 0.0;
                state_.torque_(1) = 0.0;
                reset_control_ = true;
            }
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            const HQPOutput & sol = solver_->solve(HQPData);

            state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2); 
        }
        if (ctrl_mode_ == 903){ //base backward
            if (mode_change_){
                
                stime_ = time_;
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");

                tsid_->addMotionTask(*postureTask_, 1e-5, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));  

                mode_change_ = false;
                reset_control_ = false;
            }
            if (stime_ + 5.0 > time_){
                state_.torque_(0) = -50.;
                state_.torque_(1) = -50.;
            }
            else{
                state_.torque_(0) = 0.0;
                state_.torque_(1) = 0.0;
                reset_control_ = true;
            }
            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            const HQPOutput & sol = solver_->solve(HQPData);

            state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2); 
        }
        if (ctrl_mode_ == 904){ // se3 ctrl
            if (mode_change_){
                mode_change_ = false;
                reset_control_ = false;

                if (!se3_action_.is_succeed_){
                    tsid_->removeTask("task-mobile");
                    tsid_->removeTask("task-mobile2");
                    tsid_->removeTask("task-se3");
                    tsid_->removeTask("task-posture");
                    tsid_->removeTask("task-torque-bounds");

                    tsid_->addMotionTask(*eeTask_, 1, 1);
                    tsid_->addMotionTask(*postureTask_, 1e-5, 1);
                    tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                    ROS_WARN("%f", se3_action_.duration_);

                    if (!se3_action_.is_wholebody_){
                        tsid_->addMotionTask(*mobileTask2_, 1, 0);
                        trajMobile_Cubic_->setStartTime(time_);
                        trajMobile_Cubic_->setDuration(0.001);
                        trajMobile_Cubic_->setGoalSample(robot_->getMobilePosition(data_, 5));
                        trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
                    }

                    
                    stime_ = time_;

                    H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));                    
    
                    if (se3_action_.type_ == 1 || se3_action_.type_ == 2){
                        trajEE_Cubic_->setInitSample(H_ee_ref_);
                        trajEE_Cubic_->setDuration(se3_action_.duration_);
                        trajEE_Cubic_->setStartTime(time_);
                        trajEE_Cubic_->setGoalSample(se3_action_.se3_target_);  
                    }
                    else{
                        trajEE_Constant_->setReference(se3_action_.se3_target_);
                    }

                    trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                    trajPosture_Cubic_->setDuration(1.0);
                    trajPosture_Cubic_->setStartTime(time_);
                    trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));  
                }
                else{
                    mode_change_ = false;
                }
            }    
            if (!se3_action_.is_succeed_){       
                if (se3_action_.type_ == 0) {
                    sampleEE_ = trajEE_Constant_->computeNext();
                    eeTask_->setReference(sampleEE_);
                }
                else{
                    trajEE_Cubic_->setCurrentTime(time_);
                    sampleEE_ = trajEE_Cubic_->computeNext();
                    eeTask_->setReference(sampleEE_);
                }                
                
                trajPosture_Cubic_->setCurrentTime(time_);
                samplePosture_ = trajPosture_Cubic_->computeNext();
                postureTask_->setReference(samplePosture_);
                
                if (!se3_action_.is_wholebody_){
                    trajMobile_Cubic_->setCurrentTime(time_);
                    sampleMobile_ = trajMobile_Cubic_->computeNext();
                    mobileTask2_->setReference(sampleMobile_);
                }
                const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
                const HQPOutput & sol = solver_->solve(HQPData);

                state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
                state_.torque_.head(2) = tsid_->getAccelerations(solver_->solve(HQPData)).head(2);
                if (time_ > stime_ + se3_action_.duration_ + 3.0){
                    state_.torque_.head(2).setZero();
                    //se3_action_.is_succeed_ = true;
                }
            }
            else{
                state_.torque_.setZero();
            }
        }

    }
    
    void HuskyFrankaWrapper::franka_output(VectorXd & qacc) {
        qacc = state_.torque_.tail(na_-2);
    }
    void HuskyFrankaWrapper::husky_output(VectorXd & qvel) {
        qvel = state_.torque_.head(2);
    }
    void HuskyFrankaWrapper::mass(MatrixXd & mass_mat){
        mass_mat = robot_->mass(data_).bottomRightCorner(na_-2, na_-2);
    }

    void HuskyFrankaWrapper::nle(VectorXd & nle_vec){
        nle_vec = robot_->nonLinearEffects(data_).tail(na_-2);
    }
    void HuskyFrankaWrapper::g(VectorXd & g_vec){
        g_vec = data_.g.tail(na_-2);
    }
    
    void HuskyFrankaWrapper::J(MatrixXd & Jo){
        Data::Matrix6x Jo2;
        Jo2.resize(6, 10);
        robot_->jacobianWorld(data_, robot_->model().getJointId("panda_joint7"), Jo2);
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void HuskyFrankaWrapper::base_state(Vector3d & base){
        base(0) = robot_->getMobilePosition(data_, 5).translation()(0);
        base(1) = robot_->getMobilePosition(data_, 5).translation()(1);
        base(2) = atan2(-robot_->getMobilePosition(data_, 5).rotation()(0, 1),robot_->getMobilePosition(data_, 5).rotation()(0,0));
    }

    void HuskyFrankaWrapper::ee_state(Vector3d & pos, Eigen::Quaterniond & quat){
        for (int i=0; i<3; i++)
            pos(i) = robot_->position(data_, robot_->model().getJointId("panda_joint7")).translation()(i);

        Quaternion<double> q(robot_->position(data_, robot_->model().getJointId("panda_joint7")).rotation());
        quat = q;
    }


}// namespace
