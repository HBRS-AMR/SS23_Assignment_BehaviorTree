# WS22_Assignment_BehaviorTree

## Task:
In this assignment, the motivation is to look into the implementation of behavior tree to achieve priliminary robot safety features such as low battery warning and collison avoidance. This exercise will further help in your project on bulding a wall-following robot.


## Overview:

Majority of implementations of behavior trees in robotics are using 'BehaviorTree.CPP' and 'py_trees'. 'py_trees_ros' is a wrapper for 'py_trees' to integrate with ROS. We will be using 'py_trees_ros' to implement behavior trees. ROS1 supports '0.6.0' version of 'py_trees_ros' package.

Please clone the repository of the assignment("https://github.com/HBRS-AMR/WS22_Assignment_BehaviorTree.git") on your system. Please find the following description of the files:

1. **behaviors.py**: characteristics of different behaviors are defined in this script.

2. **battery_check.py**: behavior tree to constantly check the battery status of the robot and to trigger rotation behavior once the battery level goes beyond a threshold value has to be built. Please find the example for battery_check behavior tree below:
![battery check BT](/behavior_tree/images/battery_check.png)

3. **collison_avoidance.py**: behavior tree which adds a new feature of collison avoidance to the battery_check behavior tree has to be built. Please find the example for collison_avoidance behavior tree below:
![collison avoidance BT](/behavior_tree/images/collison_battery.png)

4. **simple_publisher.py**: whenever the scripts are being run locally, then this node can be used to instantiate relevant topics such as '/cmd_vel' and '/mileage'.

## Setting up your system:

1. Please install py_trees visualiser using following command: 

    ``` 
    sudo apt install ros-noetic-rqt-py-trees ros-noetic-rqt-reconfigure

    sudo apt-get install xcb
    ```

2. Clone py_trees_ros ("git@github.com:splintered-reality/py_trees_ros.git") to your catkin workspace and checkout to the '0.6.0' branch.

3. Clone the assignment file ("https://github.com/HBRS-AMR/WS22_Assignment_BehaviorTree.git") in your system.

## Instructions to run scripts:

1. It is recommended to verify your implementation locally before testing on the robot. _simple_publisher.py_ node can be used to instantiate necessary topics.

2. While running the scripts on local machine, please make sure to run 'roscore' command in one of the terminals.

3. As the voltage values are not readily available for ROS interface, please publish the voltage values in a new terminal. 
    ```  
    Eg: 'rostopic pub /mileage std_msgs/Float32 "data: 60.0" -r 10'
    ```

4. While running on the robot, please run the launch file in 'robile_bringup' package:
    ```
    roslaunch robile_bringup robot.launch
    ```
    Once it is running, in a new terminal assign the variable 'ROS_MASTER_URI'
    ```
    export ROS_MASTER_URI=http://<robile_ip_address>:11311 && export ROS_IP=<your_system_ip_address>
    ```
    For details regarding the passwords and ip address, please refer to relevant sections in the documentation('https://robile-amr.readthedocs.io/en/latest/Tutorial/Demo%20Mapping.html')

4. To visualise the behavior tree, please run the following command:
    ```
    rqt_py_trees
    OR
    py-trees-tree-watcher
    ```

5. The values in the 'blackboard' can be displayed by running following command:
    ```
    py-trees-blackboard-watcher
    ```

## References:

1. [Behavior Trees in Action:
A Study of Robotics Applications](https://arxiv.org/pdf/2010.06256.pdf)

2. Tutorial for behavior tree implementation: http://docs.ros.org/en/noetic/api/py_trees_ros/html/
