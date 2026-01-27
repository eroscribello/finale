// Copyright (C) 2007 Francois Cauwe <francois at cauwe dot org>
// Modified for ROS 2 Action Server Integration with Multi-Threading

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    Iiwa_pub_sub() : Node("ros2_kdl_node")
    {
        using namespace std::placeholders;

        // --- PARAMETRI ---
        this->declare_parameter<double>("traj_duration", 5.0);
        this->declare_parameter<double>("total_time", 15.0);
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<std::vector<double>>("end_position", {0.1, 0.1, 0.1});
        this->declare_parameter<double>("acc_duration", 1.5);
        this->declare_parameter<std::string>("cmd_interface", "velocity");
        this->declare_parameter<std::string>("ctrl", "velocity_ctrl_null");
        this->declare_parameter<std::string>("traj_type", "linear");
        this->declare_parameter<std::string>("s_type", "trapezoidal");

        this->get_parameter("traj_duration", traj_duration_);
        this->get_parameter("total_time", total_time_);
        this->get_parameter("Kp", Kp_);
        this->get_parameter("end_position", end_position_vec_);
        this->get_parameter("acc_duration", acc_duration_);
        this->get_parameter("cmd_interface", cmd_interface_);
        this->get_parameter("ctrl", ctrl_);
        this->get_parameter("traj_type", traj_type_);
        this->get_parameter("s_type", s_type_);

        // --- MULTI-THREADING SETUP ---
        // Gruppo per le callback dei sensori (permettono l'update mentre execute() è bloccato)
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_;

        // --- INIZIALIZZAZIONE KDL ---
        setup_kdl();

        // --- TOPIC E ACTION SERVER ---
        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this,
            "/iiwa/ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, _1));

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/iiwa/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, _1), sub_options);

        MarkerPoseSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, _1), sub_options);

        std::string cmd_topic = (cmd_interface_ == "position") ? "/iiwa/iiwa_arm_controller/commands" : 
                                (cmd_interface_ == "velocity") ? "/iiwa/velocity_controller/commands" : 
                                "effort_controller/commands";
        
        cmdPublisher_ = this->create_publisher<FloatArray>(cmd_topic, 10);
        
        last_detection_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Node Ready. Multi-threaded Executor Active.");
    }

private:
    // --- MEMBRI ---
    std::shared_ptr<KDLRobot> robot_;
    KDLController controller_;
    KDLPlanner planner_;
    trajectory_point p_;
    double traj_duration_, total_time_, Kp_, acc_duration_;
    std::vector<double> end_position_vec_;
    std::string cmd_interface_, ctrl_, traj_type_, s_type_;
    KDL::JntArray joint_positions_, joint_velocities_, joint_velocities_cmd_;
    Eigen::Vector3d cPo_;
    rclcpp::Time last_detection_time_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MarkerPoseSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;

    void setup_kdl() {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Waiting for robot_description...");
        }
        
        auto parameter = parameters_client->get_parameters({"robot_description"});
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
            return;
        }

        robot_ = std::make_shared<KDLRobot>(robot_tree);
        unsigned int nj = robot_->getNrJnts();
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        controller_ = KDLController(*robot_);
        robot_->addEE(KDL::Frame::Identity());
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
    // 1. Definiamo l'ordine esatto che KDL si aspetta (quello dell'URDF)
    std::vector<std::string> ordered_names = {
        "iiwa_joint_a1", "iiwa_joint_a2", "iiwa_joint_a3", "iiwa_joint_a4", "iiwa_joint_a5", "iiwa_joint_a6", "iiwa_joint_a7"
    };

    // 2. Mappiamo ogni valore del messaggio al posto giusto nel vettore KDL
    for (size_t i = 0; i < ordered_names.size(); i++) {
        for (size_t j = 0; j < msg.name.size(); j++) {
            if (msg.name[j] == ordered_names[i]) {
                joint_positions_.data[i] = msg.position[j];
                joint_velocities_.data[i] = msg.velocity[j];
                break;
            }
        }
    }

    // 3. Ora aggiorna KDL e stampa
    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
    KDL::Frame current_f = robot_->getEEFrame();
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "STIMA KDL CORRETTA -> X: %.3f, Y: %.3f, Z: %.3f", 
        current_f.p.x(), current_f.p.y(), current_f.p.z());
}


    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped& msg) {
    // 1. Coordinate dell'oggetto nel frame ottico della camera (ArUco)
    cPo_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    last_detection_time_ = this->now();

    // 2. Otteniamo la posa del tool rispetto alla base (da KDL)
    KDL::Frame base_T_tool = robot_->getEEFrame();

    // 3. Definiamo le trasformazioni fisse dall'URDF
    // Joint camera_joint: tool0 -> camera_link (rpy: 3.14, -1.57, 0)
    KDL::Frame tool_T_cam = KDL::Frame(KDL::Rotation::RPY(3.14, -1.57, 0), 
                                       KDL::Vector(0, 0, 0));

    // Joint camera_optical_joint: camera_link -> camera_link_optical (rpy: -1.5708, 0, -1.5708)
    KDL::Frame cam_T_opt = KDL::Frame(KDL::Rotation::RPY(-1.5708, 0, -1.5708), 
                                      KDL::Vector(0, 0, 0));

    // 4. Trasformazione Finale: Base -> Tool -> Camera -> Optical -> Marker
    KDL::Vector cam_P_obj(cPo_.x(), cPo_.y(), cPo_.z());
    KDL::Vector base_P_obj = base_T_tool * tool_T_cam * cam_T_opt * cam_P_obj;

    // 5. Visualizzazione
    RCLCPP_INFO(this->get_logger(), "---------------------------------------");
    RCLCPP_INFO(this->get_logger(), "cPo (Optical): [ %f, %f, %f ]", cPo_.x(), cPo_.y(), cPo_.z());
    RCLCPP_INFO(this->get_logger(), "bPo (Base   ): [ %f, %f, %f ]", base_P_obj.x(), base_P_obj.y(), base_P_obj.z());
    
    // Salva bPo_ per usarlo nel loop di controllo vision_ctrl
    // bPo_ << base_P_obj.x(), base_P_obj.y(), base_P_obj.z();
}

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTrajectory::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void stop_robot() {
        FloatArray stop_msg;
        stop_msg.data.resize(joint_positions_.rows(), 0.0);
        cmdPublisher_->publish(stop_msg);
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        double dt = 0.02; // 50Hz
        double t = 0.0;
        rclcpp::Rate rate(1.0/dt);

        // 1. Inizializzazione Planner (Traiettoria standard)
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        Eigen::Vector3d start_p(robot_->getEEFrame().p.data);
        Eigen::Vector3d end_p(end_position_vec_[0], end_position_vec_[1], end_position_vec_[2]);
        planner_ = (traj_type_ == "linear") ? KDLPlanner(traj_duration_, acc_duration_, start_p, end_p) : KDLPlanner(traj_duration_, start_p, 0.15, acc_duration_);

        // --- LOOP CONTROLLO ---
        
        
        
        
        
        
        while (rclcpp::ok() && t < total_time_ && ctrl_ != "vision") {
            // if the client asks to cancel
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled by client");
                goal_handle->canceled(result);
                return;
            }

            if (traj_type_ == "linear") {
                if (s_type_ == "trapezoidal")
                    p_ = planner_.linear_traj_trapezoidal(t);
                else
                    p_ = planner_.linear_traj_cubic(t);
            }

            KDL::Frame cartpos = robot_->getEEFrame();

            Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));

            // publish the error as feedback
            feedback->position_error = {error(0), error(1), error(2)};
            goal_handle->publish_feedback(feedback);

            if (ctrl_ == "velocity_ctrl") {
                Vector6d cartvel;
                cartvel << p_.vel + Kp_ * error, Eigen::Vector3d::Zero();
                joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
            } else if (ctrl_ == "velocity_ctrl_null") {
                Eigen::Matrix<double, 6, 1> error_position;
                error_position << error, Eigen::Vector3d::Zero();
                joint_velocities_cmd_ = controller_.velocity_ctrl_null(error_position, Kp_);
            }

            // Publishing the command
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.assign(joint_velocities_cmd_.data.data(),
                                joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
            cmdPublisher_->publish(cmd_msg);

            t += dt;
            rate.sleep();
        }
        
        
        
        
        
        
        
        
        
        
        
        while (rclcpp::ok() &&  ctrl_=="vision") {
            if (goal_handle->is_canceling()) {
                stop_robot();
                goal_handle->canceled(result);
                return;
            }

            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            if (ctrl_ == "vision") {
                // --- LOGICA VISIONE ---
                double dt_det = (this->now() - last_detection_time_).seconds();
                if (dt_det > 0.3) {
                    stop_robot();
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Tag ArUco perso!");
                } else {
                    joint_velocities_cmd_ = controller_.vision_ctrl(Kp_, cPo_, Eigen::Vector3d(0, 0, 1));
                    publish_cmd(joint_velocities_cmd_);
                }
            } else {
                // --- LOGICA TRAIETTORIA ---
                if (t < total_time_) {
                    p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t) : planner_.linear_traj_cubic(t);
                    Eigen::Vector3d error = p_.pos - Eigen::Vector3d(robot_->getEEFrame().p.data);
                    
                    if (ctrl_ == "velocity_ctrl_null") {
                        Eigen::Matrix<double,6,1> err6; err6 << error, Eigen::Vector3d::Zero();
                        joint_velocities_cmd_ = controller_.velocity_ctrl_null(err6, Kp_);
                    } else {
                        Vector6d cartvel; cartvel << p_.vel + Kp_ * error, Eigen::Vector3d::Zero();
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                    }
                    publish_cmd(joint_velocities_cmd_);
                    t += dt;
                } else {
                    break; // Fine traiettoria temporale
                }
            }
            rate.sleep();
        }

        stop_robot();
        result->success = true;
        goal_handle->succeed(result);
    }

    void publish_cmd(const KDL::JntArray& qdot) {
        if (std::isnan(qdot.data(0))) return;
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(qdot.data.data(), qdot.data.data() + qdot.rows());
        cmdPublisher_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa_pub_sub>();
    
    // Executor multi-thread per permettere alle callback di girare durante l'action
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
