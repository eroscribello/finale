// Copyright (C) 2007 Francois Cauwe <francois at cauwe dot org>
// Modified for ROS 2 Action Server Integration

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/wait_for_message.hpp"

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

        // --- DICHIARAZIONE PARAMETRI ---
        this->declare_parameter<double>("traj_duration", 5.0);
        this->declare_parameter<double>("total_time", 10.0);
        this->declare_parameter<int>("trajectory_len", 100);
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<std::vector<double>>("end_position", {0.1, 0.1, 0.1});
        this->declare_parameter<double>("acc_duration", 1.5);
        this->declare_parameter<std::string>("cmd_interface", "velocity");
        this->declare_parameter<std::string>("ctrl", "velocity_ctrl_null");
        this->declare_parameter<std::string>("traj_type", "linear");
        this->declare_parameter<std::string>("s_type", "trapezoidal");

        // Recupero parametri
        this->get_parameter("traj_duration", traj_duration_);
        this->get_parameter("total_time", total_time_);
        this->get_parameter("trajectory_len", trajectory_len_);
        this->get_parameter("Kp", Kp_);
        this->get_parameter("end_position", end_position_vec_);
        this->get_parameter("acc_duration", acc_duration_);
        this->get_parameter("cmd_interface", cmd_interface_);
        this->get_parameter("ctrl", ctrl_);
        this->get_parameter("traj_type", traj_type_);
        this->get_parameter("s_type", s_type_);

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
            "joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, _1));

        MarkerPoseSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, _1));

        std::string cmd_topic = (cmd_interface_ == "position") ? "iiwa_arm_controller/commands" : 
                                (cmd_interface_ == "velocity") ? "velocity_controller/commands" : 
                                "effort_controller/commands";
        
        cmdPublisher_ = this->create_publisher<FloatArray>(cmd_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Node Ready. Interface: %s, Ctrl: %s", cmd_interface_.c_str(), ctrl_.c_str());
    }

private:
    // --- KDL SETUP ---
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
        
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96;
        robot_->setJntLimits(q_min, q_max);

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        desired_commands_.resize(nj, 0.0);

        controller_ = KDLController(*robot_);
        robot_->addEE(KDL::Frame::Identity());
    }

    // --- ACTION CALLBACKS ---
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTrajectory::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid; (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // --- CORE EXECUTION ---
    
    
    
    /*
    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        
        rclcpp::Rate rate(50);
        double t = 0.0;
        double dt = 1.0 / 50.0;

        // Inizializzazione planner con stato corrente
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame init_frame = robot_->getEEFrame();
        Eigen::Vector3d start_p(init_frame.p.data);
        RCLCPP_INFO(this->get_logger(), "%f, %f, %f,",end_position_vec_[0],end_position_vec_[1],end_position_vec_[2]);
        Eigen::Vector3d end_p(end_position_vec_[0], end_position_vec_[1], end_position_vec_[2]);
        
        planner_ = (traj_type_ == "linear") ? 
                   KDLPlanner(traj_duration_, acc_duration_, start_p, end_p) :
                   KDLPlanner(traj_duration_, start_p, 0.15, acc_duration_);

        while (rclcpp::ok() && t < total_time_) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                return;
            }

            // 1. Aggiorna Robot
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame current_cart_frame = robot_->getEEFrame();

            // 2. Traiettoria desiderata
            if (traj_type_ == "linear") {
                p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t) : planner_.linear_traj_cubic(t);
            } else {
                p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t) : planner_.circular_traj_cubic(t);
            }

            // 3. Calcolo Errore
            Eigen::Vector3d pos_error = p_.pos - Eigen::Vector3d(current_cart_frame.p.data);
Eigen::Vector3d ori_error = computeOrientationError(toEigen(init_frame.M), toEigen(current_cart_frame.M));
            
            feedback->position_error = {pos_error(0), pos_error(1), pos_error(2)};
            goal_handle->publish_feedback(feedback);

            // 4. Controllo Law
            if (cmd_interface_ == "velocity") {
                if (ctrl_ == "vision") {
                    joint_velocities_cmd_ = controller_.vision_ctrl(Kp_, cPo_, Eigen::Vector3d(0,0,1));
                } else if (ctrl_ == "velocity_ctrl_null") {
                    Eigen::Matrix<double,6,1> err_6d; err_6d << pos_error, ori_error;
                    joint_velocities_cmd_ = controller_.velocity_ctrl_null(err_6d, Kp_);
                } else {
                    Vector6d cart_vel; cart_vel << p_.vel + Kp_ * pos_error, ori_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cart_vel;
                }
                desired_commands_ = toStdVector(joint_velocities_cmd_.data);
            } 
            else if (cmd_interface_ == "position") {
                KDL::Frame next_f = current_cart_frame;
                next_f.p = current_cart_frame.p + (toKDL(p_.vel) + toKDL(Kp_ * pos_error)) * dt;
                robot_->getInverseKinematics(next_f, joint_positions_cmd_);
                desired_commands_ = toStdVector(joint_positions_cmd_.data);
            }

            // 5. Pubblica
            FloatArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            t += dt;
            rate.sleep();
        }

        result->success = true;
        goal_handle->succeed(result);
        stop_robot();
    }
    
    
    */
    
    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution...");

    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    double dt = 1.0 / 50.0;
    double t = 0.0;

    // 1. ATTESA DATI INIZIALI (Evita NaN se i joint_states non sono ancora arrivati)
    while (rclcpp::ok() && joint_positions_.data.norm() < 0.0001) {
        RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
        rclcpp::sleep_for(100ms);
    }

    // 2. AGGIORNAMENTO STATO ROBOT E INIZIALIZZAZIONE PLANNER
    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
    KDL::Frame init_frame = robot_->getEEFrame();
    Eigen::Vector3d start_p(init_frame.p.data);
    Eigen::Vector3d end_p(end_position_vec_[0], end_position_vec_[1], end_position_vec_[2]);

    // Inizializziamo il planner qui, altrimenti p_ sarà vuoto/NaN
    if (traj_type_ == "linear") {
        planner_ = KDLPlanner(traj_duration_, acc_duration_, start_p, end_p);
    } else {
        planner_ = KDLPlanner(traj_duration_, start_p, 0.15, acc_duration_);
    }

    // --- LOOP DI CONTROLLO TRAIETTORIA (Linear/Circular) ---
    while (rclcpp::ok() && t < total_time_ && ctrl_ != "vision") {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            stop_robot();
            return;
        }

        // Calcolo traiettoria desiderata
        if (traj_type_ == "linear") {
            p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t) : planner_.linear_traj_cubic(t);
        } else {
            p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t) : planner_.circular_traj_cubic(t);
        }

        // Aggiorna robot e calcola errore
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame current_f = robot_->getEEFrame();
        Eigen::Vector3d error = p_.pos - Eigen::Vector3d(current_f.p.data);

        // Feedback
        feedback->position_error = {error(0), error(1), error(2)};
        goal_handle->publish_feedback(feedback);

        // Calcolo comando di velocità
        if (ctrl_ == "velocity_ctrl") {
            Vector6d cartvel;
            cartvel << p_.vel + Kp_ * error, Eigen::Vector3d::Zero();
            // Utilizzo della pseudo-inversa per evitare singolarità
            joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
        } 
        else if (ctrl_ == "velocity_ctrl_null") {
            Eigen::Matrix<double, 6, 1> error_6d;
            error_6d << error, Eigen::Vector3d::Zero();
            joint_velocities_cmd_ = controller_.velocity_ctrl_null(error_6d, Kp_);
        }

        // Pubblicazione (con controllo NaN di sicurezza)
        if (!std::isnan(joint_velocities_cmd_.data(0))) {
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.assign(joint_velocities_cmd_.data.data(), 
                                joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
            cmdPublisher_->publish(cmd_msg);
        }

        t += dt;
        rclcpp::Rate rate(1.0 / dt);
        rate.sleep();
    }

    // --- LOOP DI CONTROLLO VISIONE ---
    while (rclcpp::ok() && ctrl_ == "vision") {
    // 1. Calcolo errore per il feedback
    double norm = cPo_.norm();
    if (norm < 0.001) norm = 1.0; // Evita divisione per zero
    Eigen::Vector3d s = cPo_ / norm;
    Eigen::Vector3d sd(0, 0, 1);
    Eigen::Vector3d dir_error = s - sd;

    // 2. Calcolo comando
    joint_velocities_cmd_ = controller_.vision_ctrl(1, cPo_, sd);

    // 3. Feedback e Pubblicazione
    feedback->position_error = {dir_error(0), dir_error(1), dir_error(2)};
    goal_handle->publish_feedback(feedback);

    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.assign(joint_velocities_cmd_.data.data(),
                        joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
    cmdPublisher_->publish(cmd_msg);

    // FONDAMENTALE: Lascia respirare il sistema per aggiornare cPo_
    rclcpp::Rate rate(1.0 / dt);
    rate.sleep(); 
}
    result->success = true;
    goal_handle->succeed(result);
    stop_robot();
    RCLCPP_INFO(this->get_logger(), "Execution completed.");
}
    

    void stop_robot() {
        FloatArray stop_msg;
        stop_msg.data.resize(desired_commands_.size(), 0.0);
        cmdPublisher_->publish(stop_msg);
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
        for (size_t i = 0; i < msg.position.size(); ++i) {
            joint_positions_.data[i] = msg.position[i];
            joint_velocities_.data[i] = msg.velocity[i];
        }
    }

    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped& msg) {
        cPo_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    }

    // --- MEMBRI ---
    std::shared_ptr<KDLRobot> robot_;
    KDLController controller_;
    KDLPlanner planner_;
    trajectory_point p_;

    double traj_duration_, total_time_, Kp_, acc_duration_;
    int trajectory_len_;
    std::vector<double> end_position_vec_;
    std::string cmd_interface_, ctrl_, traj_type_, s_type_;

    KDL::JntArray joint_positions_, joint_velocities_, joint_positions_cmd_, joint_velocities_cmd_;
    std::vector<double> desired_commands_;
    Eigen::Vector3d cPo_;

    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MarkerPoseSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa_pub_sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
