使用ur5e+ros2的指令记录

0. 执行桌面的start_ursim.sh(在此文件夹内有备份)
`./start_ursim.sh`

1. 运行ur_robot_driver
`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<ur_type> robot_ip:=192.168.56.101 launch_rviz:=true`

or

```ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true initial_joint_controller:=joint_trajectory_controller```

2. 使用moveit控制：
`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=<ur_type> launch_rviz:=true`


3. 使用官方的demo:
`ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py`

4. 切换控制器：
`ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller`# ur5e_basic_ros2
