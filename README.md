# ROS1 Bridge

The ``ros1_bridge`` package allows you to map topics, services, and actions between ROS1 and ROS2, enabling seamless communication and integration between the two ROS distributions. This is particularly useful when you have existing ROS1 code that you want to leverage in a ROS2 environment or when you need to integrate ROS1 and ROS2 components in a single system.


***

# Creating ROS1 and ROS2 Workspaces


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



# Arduino Robot Arm Project:
we will be working on [Arduino Robot Arm](https://github.com/smart-methods/arduino_robot_arm).

## 1. Install Arduino Robot Arm Aackage in ROS Noetic:


1. Open a **Terminal** & Source the ROS 1 Noetic: 
   > Ctrl + Alt + T
```
  $ source /opt/ros/noetic/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "noetic".
    


2. Dependencies:
   - Install moveit_ros_planning:
```
 $ sudo apt-get install ros-noetic-moveit
```

-(If Error Encountered ): Rebuild the Workspace 

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

1. Open a **Terminal** & Source:
```
  $ source /opt/ros/noetic/setup.bash
  $ source /opt/ros/foxy/setup.bash
```
    
2. Create the Workspace:
```
   $ cd
   $ mkdir -p ~/ros1_bridge_ws/src
   $ cd ~/ros1_bridge_ws/src
   $ git clone -b foxy https://github.com/ros2/ros1_bridge.git
```
 

3.  Build the ROS1 Bridge Package:
```
   $ cd ~/ros1_bridge_ws
   $ colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
   $ source install/local_setup.bash
```


4. Source ROS1 and ROS2:
   
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
   $ Install python3-colcon-common-extensions and python3-ament-package:
   $ sudo apt update
   $ sudo apt install python3-colcon-common-extensions python3-ament-package
```

   - Rebuild the ros1_bridge:
```
   $ cd ~/ros1_bridge_ws
   $ colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
```


5. Test the bridge:
```
   $ cd
   $ source install/local_setup.bash
   $ ros2 run ros1_bridge dynamic_bridge --print-pairs
```

6. Open a New **Terminal** & Source:
```
  $ source /opt/ros/noetic/setup.bash
  $ source ~/catkin_ws/devel/setup.bashash
```
   
   - Launch the Arduino_robot_arm package:
```
   $ roslaunch robot_arm_pkg check_motors.launch
```

   - You should see this:
 <img src="https://github.com/user-attachments/assets/b874fdaa-0ffe-4bb0-af24-a2bf2659360e" width="600" height="230">


7. Open a New **Terminal** & run:
```
   $ rostopic list
```
   - To View all ROS nodes that are currently running in the ROS environment.

     
***


## 3. Running ROS 1 Bridge:

After launching the check_motors.launch file and have the robot arm running in ROS1, you need to set up the ros1_bridge and run it to ensure communication between ROS1 and ROS2.

1. Open a **Terminal** & run:
   > Ctrl + Alt + T
```
   $ source /opt/ros/noetic/setup.bash
   $ source ~/catkin_ws/devel/setup.bash
   $ cd  ros1_bridge_ws/
   $ source install/setup.bash
   $ ros2 run ros1_bridge dynamic_bridge
```
   - Bridge is Created.

2. Open a **Terminal** & Echo /joint_states topic in ROS2:
```
   $ source /opt/ros/foxy/setup.bash
   $ ros2 topic echo /joint_states sensor_msgs/msg/JointState
```

***
