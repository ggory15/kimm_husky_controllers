//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Int16.h"

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// for hqp controller
#include <kimm_hqp_controller/robot/robot_wrapper.hpp>
#include <kimm_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp>
#include <kimm_hqp_controller/tasks/task_se3_equality.hpp>
#include <kimm_hqp_controller/tasks/task_joint_posture.hpp>
#include <kimm_hqp_controller/tasks/task_joint_bound.hpp>
#include <kimm_hqp_controller/tasks/task_mobile_base.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_euclidian.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_se3.hpp>
#include <kimm_hqp_controller/solver/solver_HQP_factory.hxx>
#include <kimm_hqp_controller/solver/util.hpp>
#include <kimm_hqp_controller/math/util.hpp>

// service 
#include "kimm_joint_planner_ros_interface/action_joint_path.h"
#include "kimm_se3_planner_ros_interface/action_se3_path.h"
#include "kimm_path_planner_ros_interface/action_mobile_path.h"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;

typedef struct State {   
    VectorXd q_;
    VectorXd v_;
    VectorXd dv_;
    VectorXd torque_;
} state;   
typedef struct Mob{
    MatrixXd lambda_inv_;
    MatrixXd lambda_, mass_, mass_inv_;
    MatrixXd J_trans_;
    MatrixXd J_trans_inv_;
    VectorXd d_force_;
    VectorXd d_torque_, torque_d_, torque_d_prev_;
    VectorXd g_, nle_, coriolis_;
    double gamma_, beta_;
    VectorXd p_k_prev_, p_k_, alpha_k_;
} mob;

namespace RobotController{
    class HuskyFrankaWrapper{
        public: 
            HuskyFrankaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node);
            ~HuskyFrankaWrapper(){};

            void initialize();
            void ctrl_update(const int& ); // msg for chaning controller
            void franka_update(const sensor_msgs::JointState::ConstPtr&); // franka state update
            void franka_update(const Vector7d&, const Vector7d&); // franka state update

            void husky_update(const sensor_msgs::JointState::ConstPtr&); // husky state update
            void husky_update(const Vector3d&, const Vector3d&, const Vector2d&, const Vector2d&); // husky state update

            void compute(const double &); // computation by hqp controller
            void franka_output(VectorXd & qacc); // joint torque of franka 
            void husky_output(VectorXd & qvel); // joint velocity of husky

            void mass(MatrixXd & mass_mat);
            void nle(VectorXd & nle_vec);
            void J(MatrixXd & J);
            void g(VectorXd & g_vec);

            void ee_state(Vector3d & pos, Eigen::Quaterniond & quat);
            void base_state(Vector3d & base);

            int ctrltype(){
                return ctrl_mode_;
            }
            void state(State & state_robot){
                state_robot = state_;
            }

        private:
            bool issimulation_, mode_change_, update_weight_;
            double stime_, time_, node_index_, node_num_, prev_node_;
            std::string robot_node_;
            State state_;
        
            int ctrl_mode_;
            Eigen::VectorXd q_ref_;
            pinocchio::SE3 H_ee_ref_, H_mobile_ref_;

            //hqp
            std::shared_ptr<kimmhqp::robot::RobotWrapper> robot_;
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::shared_ptr<kimmhqp::InverseDynamicsFormulationAccForce> tsid_;
            
            std::shared_ptr<kimmhqp::tasks::TaskJointPosture> postureTask_;
            std::shared_ptr<kimmhqp::tasks::TaskSE3Equality> eeTask_;
            std::shared_ptr<kimmhqp::tasks::TaskJointBounds> torqueBoundsTask_;
            std::shared_ptr<kimmhqp::tasks::TaskMobileEquality> mobileTask_, mobileTask2_;

            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianCubic> trajPosture_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianConstant> trajPosture_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianTimeopt> trajPosture_Timeopt_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Cubic> trajEE_Cubic_, trajMobile_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Constant> trajEE_Constant_, trajMobile_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Timeopt> trajEE_Timeopt_, trajMobile_Timeopt_;            

            kimmhqp::trajectory::TrajectorySample sampleMobile_, sampleEE_, samplePosture_;

            kimmhqp::solver::SolverHQPBase * solver_;
            
            int na_, nq_, nv_;

            //service
            kimm_path_planner_ros_interface::action_mobile_path action_mobile_srv_; 
            kimm_joint_planner_ros_interface::action_joint_path action_joint_srv_; 
            kimm_se3_planner_ros_interface::action_se3_path action_se3_srv_; 
            ros::ServiceClient joint_action_client_, se3_action_client_, mobile_action_client_;

            //ros
            ros::NodeHandle n_node_;
    };
} // namespace


