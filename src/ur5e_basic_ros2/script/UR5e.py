import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
import rclpy.node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
import PyKDL as kdl
from PyKDL import ChainFkSolverPos_recursive, ChainJntToJacSolver
from dataclasses import dataclass

# @dataclass
# class JntTrajectory:
#     index: int
#     position: list # List[double]
#     velocity: list # List[double]
#     duration: Duration

# @dataclass
# class Jnt
 

class UR5e(rclpy.node.Node):

    def __init__(self, controller_type: str):
        super().__init__("UR5e")

        # 配置控制器名和关节名
        self.declare_parameter("controller_name", controller_type)
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
        )
        
        # 获取参数
        self.controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        # 关节数校验
        if self.joints is None or len(self.joints) == 0:
            raise ValueError("No joint names specified")
        
        # 初始化动作客户端
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_type)
        self.get_logger().info("Waiting for action server on {}".format(controller_type))
        self._action_client.wait_for_server()

        # 关节控制模式
        if controller_type == "scaled_joint_trajectory_controller":
            self.parse_trajectories()
            self.i = 0 # 当前执行的轨迹索引号
            self._send_goal_future = None
            self._get_result_future = None
            self.execute_next_trajectory()

    def execute_trajectory(self, trajectory: kdl.JntArray = None):
        pass
            

if __name__ == "__main__":
    rclpy.init()
    ur5e = UR5e("scaled_joint_trajectory_controller")
    ur5e.execute_trajectory()