#include "UR5e.hpp"




UR5e::UR5e(const std::string nodeName, std::string controllerName):
    Node(nodeName), nodeName(nodeName), controllerName(controllerName)
{
    this->declare_parameter("controller_name", this->controllerName);  // 往参数服务器里添加参数controller_name
    this->declare_parameter<std::vector<std::string>>("jointNames", {  // 往参数服务器里添加参数jointNames
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    });

    this->jointNames = this->get_parameter("jointNames").as_string_array();  // 把jointNames放到类里
    
    // 判断是否成功读取jointNames
    if (this->jointNames.empty()) {
        RCLCPP_ERROR(get_logger(), "Parameter 'jointNames' is required but not set.");
        exit(1);
    }

    // 实例化 参数服务器 读取机器人urdf描述文件
    this->paramClient_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/scaled_joint_trajectory_controller");
    while (!this->paramClient_->wait_for_service(std::chrono::seconds(3))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return ;
          }
          RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto parameter = this->paramClient_->get_parameters({"robot_description"});
    std::string robot_description = parameter[0].value_to_string();
    

    // 启动运动学求解器
    if (robot_description.empty())
        RCLCPP_ERROR(this->get_logger(), "robot_description is empty!");
    else{
        RCLCPP_INFO(this->get_logger(), "Successfully read robot_description!");
        this->kinDynSolver_ = std::make_shared<KinDyn>(robot_description, "base_link", "tool0", 0.005);
    }

    // 动作客户端设置
    this->actionClient_ = rclcpp_action::create_client<FlwJntTra>(
        this, 
        this->controllerName + "/follow_joint_trajectory");

    if (!this->actionClient_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    // this->MoveTo(traj, 0.5, false);
}

void UR5e::MoveTo(JntTraPoint target, double time, bool wait_for_D){
    auto targetTar = trajectory_msgs::msg::JointTrajectory();
    targetTar.points.push_back(target);
    targetTar.joint_names = this->jointNames;
    
    this->MoveTo(targetTar, time, wait_for_D);
}

void UR5e::MoveTo(JntTra target, double time, bool wait_for_D)
{
    RCLCPP_INFO(this->get_logger(), "Move to target");
    std::cout << "jntPos: " << this->jntPosCur(0) << std::endl;
    auto goalPoint = FlwJntTra::Goal();
    goalPoint.trajectory = target;

    auto chrono_duration = std::chrono::duration<double>(time);
    goalPoint.goal_time_tolerance = rclcpp::Duration(chrono_duration);
    auto send_goal_options = rclcpp_action::Client<FlwJntTra>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&UR5e::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&UR5e::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&UR5e::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    this->actionClient_->async_send_goal(goalPoint, send_goal_options);

    // if (wait_for_D)
    //     rclcpp::Duration(chrono_duration).sleep();
}

void UR5e::goal_response_callback(const rclcpp_action::ClientGoalHandle<FlwJntTra>::SharedPtr & future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void UR5e::feedback_callback(rclcpp_action::ClientGoalHandle<FlwJntTra>::SharedPtr goal_handle, const std::shared_ptr<const FlwJntTra::Feedback> feedback) {
    
    std::vector<std::string> joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint jntMsg;
    jntMsg = feedback->actual;
    joint_names = feedback->joint_names;
    
    // std::cout << "joint position: ";
    // for (int i=0; i < 6; i++)
    //     std::cout << jntPos.positions[i] << " ";
    // std::cout << std::endl;

    KDL::JntArray jntPos, jntVel, jntAcc, jntCur;

    jntPos.resize(this->jointSize);
    jntVel.resize(this->jointSize);
    // jntCur.resize(this->jointSize);
    // jntAcc.resize(this->jointSize);

    int n = joint_names.size();

    for (int i = 0; i < this->jointSize; ++i) {
        int x = 0;
        for (; x<n; ++x)
        {
            if(joint_names[x] == this->jointNames[i]){
                jntPos(i) = jntMsg.positions[x];
                jntVel(i) = jntMsg.velocities[x];
                // jntCur(i) = jntMsg.effort[x];
                // jntAcc(i) = jntMsg.accelerations[x];
                break;
            }
        }
        if (x==n) {
            RCLCPP_ERROR(this->get_logger(), "Error, joint name not found");
            return ;
        }
    }

    this->jntPosCur = jntPos;
    this->jntVelCur = jntVel;
    // this->jntCurCur = jntCur;
    // this->jntAccCur = jntAcc;

    if (kinDynSolver_ && kinDynSolver_->fkSolver_) {
        this->kinDynSolver_->fkSolver_->JntToCart(jntPosCur, FrameCur);
    } else {
        RCLCPP_ERROR(this->get_logger(), "kinDynSolver not initialized");
    }
    std::cout << "joint position: ";
    for (int i=0; i < 6; i++)
        std::cout << this->jntPosCur(i) << " ";
    std::cout << std::endl;

    std::cout << "end position: ";
    for (int i = 0; i < 3; i++)
        std::cout <<this->FrameCur.p.data[i] << " ";
    std::cout << std::endl;  
}


void UR5e::result_callback(const rclcpp_action::ClientGoalHandle<FlwJntTra>::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            rclcpp::shutdown();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            rclcpp::shutdown();
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            rclcpp::shutdown();
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            rclcpp::shutdown();
            return;
    }
}





int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<UR5e> ur5e = std::make_shared<UR5e>("ur5e",  "/scaled_joint_trajectory_controller");
    auto traj = trajectory_msgs::msg::JointTrajectory();
    std::vector<std::string> jointNames ={
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    };
    traj.joint_names = jointNames;
    trajectory_msgs::msg::JointTrajectoryPoint point1;
    point1.positions = {0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18};
    point1.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point1.time_from_start = rclcpp::Duration(4, 0); // 注意这里需要秒和纳秒，所以使用两个参数构造函数
    traj.points.push_back(point1);
    
    // 创建第二个轨迹点
    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.positions = {-0.195016, -1.70093, 0.902027, -0.944217, -1.52982, -0.195171};
    point2.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point2.time_from_start = rclcpp::Duration(8, 0);
    traj.points.push_back(point2);

    ur5e->MoveTo(traj, 0.5, false);

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(ur5e);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}