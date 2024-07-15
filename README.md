# ROS1-Bridge

## 1. Create ROS1 Noetic Workspace:
   
1. Open a **Terminal** & Source the ROS 1 Noetic: 
   > Ctrl + Alt + T
```
  $ source /opt/ros/noetic/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "noetic".
  - Create the Workspace Directory:
```
  $ cd
  $ mkdir catkin_ws
```

2. Create The Source Folder & Build the Workspace:  

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

1. Open a new Terminal & Source the ROS 2 Foxy & Create the Workspace Directory::
```
  $ source /opt/ros/foxy/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "foxy".
   
```
  $ cd
  $ mkdir ros2_ws
```

2. Create The Source Folder:

  ```
    $ cd ros2_ws/
    $ mkdir src
  ```

3. Install _colcon_:

 ```
   $ sudo apt update
   $ sudo apt install python3-colcon-common-extensions
```

4. Build the Workspace: 

 ```
   $ cd ~/ros2_ws/
   $ colcon build
   $ source install/local_setup.bash
```

5. Source  Workspace: 

 ```
   $ cd ~/ros2_ws/install/
   $ source setup.bash  
  ```

***



# Arduino Robot Arm Project


## 1. Install Arduino Robot Arm Aackage in ROS Noetic:


1. Open a **Terminal** & Source the ROS 1 Noetic: 
   > Ctrl + Alt + T
```
  $ source /opt/ros/noetic/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "noetic".
    


2. Dependencies (noetic distro):
```
 $ cd catkin_ws
 $ sudo apt install python3-rosdep2
 $ sudo apt-get install ros-noetic-moveit
 $ sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
 $ sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher
 $ sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
```


-(If encountered an error): Rebuild the Workspace 

```
   $ cd ~/catkin_ws
   $ catkin_make
```


3. Install the Arduino Robot Arm Package:
```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/smart-methods/arduino_robot_arm.git
   $ cd ..
   $ catkin_make
   $ source devel/setup.bash
```



***


## 2. Create and Set Up ros1_bridge Workspace:

1. Create the Workspace:
```
   $ mkdir -p ~/ros1_bridge_ws/src
   $ cd ~/ros1_bridge_ws/src
   $ git clone -b foxy https://github.com/ros2/ros1_bridge.git
```

2.  Build the ROS1 Bridge Package:
```
   $ cd ~/ros1_bridge_ws
   $ colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
   $ source install/local_setup.bash
```


3. Source ROS1 and ROS2:
   
   - Ensure All Required ROS 2 Packages are Installed:
```
   sudo apt update
   sudo apt install ros-foxy-rmw-cyclonedds-cpp ros-foxy-rmw-fastrtps-cpp ros-foxy-rmw-implementation
```

   - Source ROS1 and ROS2:
```
   source /opt/ros/noetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   source /opt/ros/foxy/setup.bash
   source ~/ros2_ws/install/local_setup.bash
```

   - _If Error Encountered_:
```
   Install python3-colcon-common-extensions and python3-ament-package:
   sudo apt update
   sudo apt install python3-colcon-common-extensions python3-ament-package
```

   - Rebuild the ros1_bridge:
```
   cd ~/ros1_bridge_ws
   colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
```


4. Test the bridge:
```
   source install/local_setup.bash
   ros2 run ros1_bridge dynamic_bridge --print-pairs
```

5. Launch the Arduino_robot_arm package:
```
   source /opt/ros/noetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   roslaunch robot_arm_pkg check_motors.launch
   rostopic list
```


***


## 3. Running ros1_bridge

After launching the check_motors.launch file and have the robot arm running in ROS1, you need to set up the ros1_bridge and run it to ensure communication between ROS1 and ROS2.

1. Open a **Terminal** & run this:
   > Ctrl + Alt + T
```
   $ source install/setup.bash
   $ ros2 run ros1_bridge dynamic_bridge
```

2. Echo /joint_states topic in ROS2:
```
   source /opt/ros/foxy/setup.bash
   ros2 topic echo /joint_states sensor_msgs/msg/JointState
```

***
