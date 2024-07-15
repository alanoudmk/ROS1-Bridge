# ROS1-Bridge

## 1. Create ROS1 Noetic Workspace:
   
1. Open a **Terminal** & Create the Workspace Directory: : 
   > Ctrl + Alt + T
```
  $ source /opt/ros/noetic/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "noetic".
   
```
  $ cd
  $ mkdir catkin_ws
```

2. Create The Source Folder: 

  ```
    $ cd catkin_ws/
    $ mkdir src
    $ catkin_make
    $ source devel/setup.bash
  ```

3. Source Catkin Workspace: 

 ```
   $ cd devel/
   $ source setup.bash 
  ```



***



## 2. Create ROS 2 Foxy Workspace:

1. Open a new Terminal & Source the ROS 2 Foxy:
```
  $ source /opt/ros/foxy/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "foxy".

3. Create the ROS 2 workspace directory structure & Initialize the Workspace::
  ```
   $ mkdir -p ros2_ws/src
   $ ros2 pkg create --build-type ament_cmake my_package_cpp --dependencies rclcpp
```

4. Open a new Terminal & Initialize the Workspace:
```
   $ source /opt/ros/foxy/setup.bash
   $ cd ros2_ws/src/
   $ ros2 pkg create --build-type ament_python my_package_py 
```
  - This will create two new packages in the src directory of your workspace.
 
5. Build the ROS 2 workspace:
   - run this in the First Terminal:
```
  $ cd ..
  $ colcon build
```

6. Source the workspace :
```
  $ source install/setup.bash
```

  - Go to the seconed Terminal and source it as well:
```
  $ cd ..
  $ source install/setup.bash
```
  - This will set up the necessary environment variables for your ROS 2 Foxy workspace.



***




## 3. Install Arduino Robot Arm Aackage in ROS Noetic:

1. Update your package index:
```
  sudo apt update
```
- Install colcon:
```
  sudo apt install python3-colcon-common-extensions
 ``` 
- Dependencies:
```
 $ cd catkin_ws
 $ sudo apt install python3-rosdep2
 $ sudo apt-get install ros-noetic-moveit
 $ sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
 $ sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher
 $ sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
```


- Rebuild the Workspace (if encountered an error):
```
  cd ~/catkin_ws
  catkin_make
```

2. Install the Arduino Robot Arm Package:
```
  cd ~/catkin_ws/src
  git clone https://github.com/smart-methods/arduino_robot_arm.git
  cd ..
  catkin_make
  source devel/setup.bash
```

3. Create and Set Up ros1_bridge Workspace:
```
 $ mkdir -p ~/ros1_bridge_ws/src 
 $ cd ~/ros1_bridge_ws/src 
 $ git clone -b foxy https://github.com/ros2/ros1_bridge.git 
```

4. Build the ROS1 Bridge Package:
```
 $ cd ~/ros1_bridge_ws
 $ colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
 $ source install/local_setup.bash
```

5. Source ROS1 and ROS2 setup bash files
  - Ensure All Required ROS 2 Packages are Installed:
```
  sudo apt update
  sudo apt install ros-foxy-rmw-cyclonedds-cpp ros-foxy-rmw-fastrtps-cpp ros-foxy-rmw-implementation
```

6. Source ROS1 and ROS2:
```
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  source /opt/ros/foxy/setup.bash
  source ~/ros2_ws/install/local_setup.bash
```
  - If error encountered:
  - Install python3-colcon-common-extensions and python3-ament-package:
```
  sudo apt update
  sudo apt install python3-colcon-common-extensions python3-ament-package
```

7. Rebuild the ros1_bridge:
```
  cd ~/ros1_bridge_ws
  colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
```

8. Test the bridge:
```
  source install/local_setup.bash
  ros2 run ros1_bridge dynamic_bridge --print-pairs
```
9. Launch the Arduino_robot_arm package:
```
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  roslaunch robot_arm_pkg check_motors.launch
```
```
  rostopic list
```
***
