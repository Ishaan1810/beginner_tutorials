## ROS 2 Programming exercise
Creating a subscriber and publisher in ROS2



### Building the ROS package
```bash
# Source ROS
source /opt/ros/humble/setup.bash
# Navigate to workspace
cd ~/ros2_ws/src
# Navigate to home directory
cd ..
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package 
colcon build --packages-select beginner_tutorials
# Source the package
. install/setup.bash
# Run the publisher
ros2 run beginner_tutorials talker
# Run the subscriber
ros2 run beginner_tutorials listener 
```

### CppCheck
```bash
# If you need to install cppcheck, do
sudo apt install cppcheck

# Run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" )
```

### CppLint
```bash
# You may need to install cpplint:
sudo apt install cpplint

# run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```


[INFO] [1699397210.604083570] [Subscriber]: Hey, I heard: 'Hey there, ROS2 here! 9'
[INFO] [1699397211.103319098] [Subscriber]: Hey, I heard: 'Hey there, ROS2 here! 10'
[INFO] [1699397211.603818082] [Subscriber]: Hey, I heard: 'Hey there, ROS2 here! 11'

```
