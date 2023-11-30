# beginner_tutorials
A repository for ros2 humble 

### Author
Ishaan Parikh 

### Building the ROS package
```bash
# Source ROS
source /opt/ros/humble/setup.bash
# Navigate to workspace
cd ~/ros2_ws/src
git clone https://github.com/Ishaan1810/beginner_tutorials.git

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
```
### Bags 
```bash
# make bags directory
mkdir bags

# to run the bag
ros2 bag record chatter

# to review the recording
ros2 bag info recording_bag_name

### Custom Launch
```bash
ros2 launch beginner_tutorials custom_launch.yaml frequency:=1ros2 launch beginner_tutorials custom_launch.yaml frequency:=1
```
### tf2_frames
```bash
# to run the talker frame
ros2 run beginner_tutorials talker ishaan 0 1 0 0 0 0

# to echo the frame 
ros2 run tf2_ros tf2_echo world ishaan

# results
At time 0.0
- Translation: [0.000, 1.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000  0.000
  0.000  1.000  0.000  1.000
  0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000

```
### test 
```bash
# command to run gtest
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials

```

### CppCheck
```bash
# install cppcheck
sudo apt install cppcheck

# Run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" )
```

### CppLint
```bash
# install cpplint:
sudo apt install cpplint

# run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```

## rqt_console log output
![Capture1](https://github.com/Ishaan1810/beginner_tutorials/assets/20563798/9c286cf4-02d9-4509-b62b-b63ceac02049)
![Capture](https://github.com/Ishaan1810/beginner_tutorials/assets/20563798/70836b51-536e-445a-8649-a1c4c7af508a)

Verfied Doxygen commenting for each file


