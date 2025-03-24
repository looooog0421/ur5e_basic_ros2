#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <trac_ik/trac_ik.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


class KinDyn {
public:

    
    KinDyn(const std::string &_urdf_param, const std::string &_chain_start, 
           const std::string &_chain_end, double timeout);

    void Jnt2End(sensor_msgs::msg::JointState &jointState);

    std::vector<std::string> jointName_; // 关节名称
 
    std::string urdf_path; // urdf路径


    Eigen::VectorXd q, dq, ddq;


    std::shared_ptr<TRAC_IK::TRAC_IK> trac_ikSolver_;
    bool valid_;
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fkSolver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jacSolver_;
private:
};