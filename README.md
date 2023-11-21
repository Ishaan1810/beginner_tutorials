##Building the Ros Package
# Source ROS
source /opt/ros/humble/setup.bash
# Navigate to workspace
cd ~/ros2_ws/src
git clone https://github.com/mvboiii/beginner_tutorials.git

# Navigate to home directory
cd ..
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package 
colcon build --packages-select beginner_tutorials
# Source the package
. install/setup.bash
# Run the publisher
ros2 run beginner_tutorials talker --ros-args --log-level debug
# Run the subscriber
ros2 run beginner_tutorials listener --ros-args --log-level debug
# Run server_client
ros2 run beginner_tutorials server_client
