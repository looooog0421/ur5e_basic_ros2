#include <kin_dyn.hpp>

KinDyn::KinDyn(const std::string &_urdf_param,
    const std::string &_chain_start, const std::string &_chain_end, double timeout)
{
    this->trac_ikSolver_ = std::make_shared<TRAC_IK::TRAC_IK>(_chain_start, _chain_end, _urdf_param, timeout, 1e-5);
    valid_ = this->trac_ikSolver_->getKDLChain(this->chain_);
    fkSolver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(this->chain_);
    jacSolver_ = std::make_shared<KDL::ChainJntToJacSolver>(this->chain_);
}

