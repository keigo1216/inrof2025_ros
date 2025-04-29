# inrof2025_ros

# install
## enviroment
- ubuntu22.04 LTS

## ros2 humble install
[offical install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

set `source /opt/ros/humble/setup.zsh` to `~/.zshrc`
```zsh
echo 'source /opt/ros/humble/setup.zsh' >> ~/.zshrc
```

create ros2 workspace and clone this repository
```bash
cd 
mkdir ros_ws/src
cd ros_ws/src
git clone git@github.com:keigo1216/inrof2025_ros.git
```

[install rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
```bash
sudo apt-get install python3-rosdep
```
```bash
sudo rosdep init
rosdep update
```
```bash
rosdep install --from-paths src -y --ignore-src
```

[install colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

```bash
sudo apt install -y python3-colcon-common-extensions
```

# env settings
Set `ROS_DOMAIN_ID` for Distributed processing and restart ros2 demon
```bash
echo 'export ROS_DOMAIN_ID=1' >> ~/.bashrc
ros2 daemon stop
ros2 daemon start
```

```bash
echo 'export GAZEBO_MODEL_PATH=~/ros_ws/src/inrof2025_ros/models/field:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ros_ws/src/inrof2025_ros/models/field:$GAZEBO_MODEL_PATH' >> ~/.bashrc 
```

# control keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```