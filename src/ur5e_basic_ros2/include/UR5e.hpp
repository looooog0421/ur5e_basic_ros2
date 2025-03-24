// #ifndef UR5e_HPP
// #define UR5e_HPP
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

#include <rclcpp_action/rclcpp_action.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/jacobian.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/algorithm/crba.hpp"
// #include "pinocchio/algorithm/centroidal.hpp"
// #include "pinocchio/algorithm/center-of-mass.hpp"
// #include "pinocchio/algorithm/aba.hpp"

#include <boost/filesystem.hpp>
#include <kin_dyn.hpp>
class UR5e : public rclcpp::Node {
public:
    using FlwJntTra = control_msgs::action::FollowJointTrajectory;
    using JntTraPoint = trajectory_msgs::msg::JointTrajectoryPoint;
    using JntTra = trajectory_msgs::msg::JointTrajectory;

    
    UR5e(const std::string nodeName, std::string controllerName);

    const std::string nodeName{"nodeUR5e"};
    std::string controllerName; 
    std::vector<std::string> jointNames;

    std::string urdf_param;
    std::string ur_prefix;
    std::shared_ptr<KinDyn> kinDynSolver_;

    rclcpp_action::Client<FlwJntTra>::SharedPtr actionClient_;
    rclcpp::SyncParametersClient::SharedPtr paramClient_;

    JntTraPoint goalPoint;

    void MoveTo(JntTraPoint target, double time, bool wait_for_D);
    void MoveTo(JntTra target, double time, bool wait_for_D);
    
    KDL::JntArray getCurrentJointState(){return jntPosCur;}

private:
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<FlwJntTra>::SharedPtr & future);
    void feedback_callback(rclcpp_action::ClientGoalHandle<FlwJntTra>::SharedPtr goal_handle, const std::shared_ptr<const FlwJntTra::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<FlwJntTra>::WrappedResult &result);

    // void subJointState(const sensor_msgs::msg::JointState &state);

    int jointSize{6};

    KDL::JntArray jntPosCur{jointSize};
    KDL::JntArray jntVelCur{jointSize};
    KDL::JntArray jntCurCur{jointSize}; // 电流
    KDL::JntArray jntAccCur{jointSize};
    
    KDL::Frame FrameCur;
    bool flag{false};
    bool joint_state_received_{false}; // 添加此行
    rclcpp::CallbackGroup::SharedPtr cbkGrpSub;                                // 回调函数组 用于上层通讯 Reentrant 模式
    rclcpp::CallbackGroup::SharedPtr cbkGrpAct;                                // 回调函数组 用于上层通讯 Reentrant 模式
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJntSta;           // subscriber hmcSta
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> subOptions;  // options 用于接收者 Reentrant 模式

};


// void UR5e::subJointState(const sensor_msgs::msg::JointState &state)
// {
//     // if(!joint_state_received_){
//         std::cout << "enter sub" << std::endl;
//         joint_state_received_ = true; // 更新标志
//     // }
    
//     // // flag = true;
//     // KDL::JntArray jntPos;
//     // KDL::JntArray jntVel;
//     // KDL::JntArray jntCur;
//     // jntPos.resize(this->jointSize);
//     // jntVel.resize(this->jointSize);
//     // jntCur.resize(this->jointSize);

//     // int n = state.name.size();

//     // for (int i = 0; i < this->jointSize; ++i) {
//     //     int x = 0;
//     //     for (; x < n; ++x)
//     //     {
//     //         if (state.name[x] == this->jointNames[i])
//     //         {
//     //             jntPos(i) = state.position[x];
//     //             jntVel(i) = state.velocity[x];
//     //             jntCur(i) = state.effort[x];
//     //             // std::cout << "state.position[" << i << "]: " << jntPos(i) << std::endl;
//     //             // std::cout << "jointNames[" << i << "]: " <<jointNames[i];
//     //             break;
//     //         }
//     //     }
//     //     if (x == n) {
//     //         RCLCPP_ERROR(this->get_logger(), "Error,  joint name not found.");
//     //         return;
//     //     }
//     // }
//     // jntPosCur = jntPos;
//     // jntVelCur = jntVel;
//     // jntCurCur = jntCur;
//     // // std::cout << "jntPosCur: " << jntPosCur(0) << std::endl;
//     // if (kinDynSolver_ && kinDynSolver_->fkSolver_) {

//     //     this->kinDynSolver_->fkSolver_->JntToCart(jntPosCur, FrameCur);
        
//     //     } else {
        
//     //     RCLCPP_ERROR(this->get_logger(), "kinDynSolver not initialized");
        
//     //     }

// }