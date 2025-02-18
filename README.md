# inrof2025_ros

# env settings
```bash
echo 'export GAZEBO_MODEL_PATH=~/ros_ws/src/inrof2025_ros/models/field:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ros_ws/src/inrof2025_ros/models/field:$GAZEBO_MODEL_PATH' >> ~/.bashrc 
```

# control keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```