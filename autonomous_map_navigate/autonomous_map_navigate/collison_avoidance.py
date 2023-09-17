#!/usr/bin/env python3

import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
from sensor_msgs.msg import LaserScan
import rclpy
import sys
from autonomous_map_navigate.behaviors import *


def create_root() -> pt.behaviour.Behaviour:
    """
    Method to structure the behavior tree to monitor battery status and start rotation if battery is low.
    Also, the robot will stop if it detects an obstacle in front of it.
    
    The "collison_avoidance" behavior tree extends the "battery_monitor" behavior tree by adding a new feature
    to avoid collison with obstacles. Whenever the robot is about to collide with an object, the robot will
    automatically stop, overriding the input commands. The robot can be controlled either by joystick,
    where the command is published on the '/joy' topic or by command that is published on '/cmd_vel' topic.
    The laser scan data will be stored in blackboard by reading '/scan' topic. When an obstacle
    is detected, the 'stop_motion' behavior will be executed. The stop_motion behavor is prioritized over
    the rotate behavior.
    """

    ## define nodes and behaviors

    # define root node
    root = pt.composites.Parallel(
        name="root",
        policy=pt.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )    

    """
    Create a sequence node called "Topics2BB" and a selector node called "Priorities"    
    """
    ### YOUR CODE HERE ###
    
    """
    Create sequence node called "NavigationStack"
    """
    navigation_stack = pt.composites.Sequence("NavigationStack") 

    """
    Using the battery_status2bb class, create a node called "Battery2BB" which subscribes to the topic "/battery_voltage"
    and the laser_scan_2bb class, create a node called "LaserScan2BB" which subscribes to the topic "/scan"
    """    
    ### YOUR CODE HERE ###

    """
    Using the rotate class, create a node called "rotate_platform", and using the stop_motion class, create a node called "stop_platform"
    """
    ### YOUR CODE HERE ###

    """
    Create mapping and localisation sequence nodes and coresponding child nodes for mapping and localisation
    """
    mapping = pt.composites.Sequence("Map")
    localisation = pt.composites.Sequence("Localisation")

    gmapping = launchNodes(name="GMapping", launch_file="online_async_launch.py", mapping_time_out=300, map_name='map') # mapping_time_out (in seconds), generally set to desired wall following time
    amcl = launchNodes(name="AMCL", launch_file="localization_launch.py")

    """
    Read the 'battery_low_warning' and 'collison_warning' from the blackboard and set a decorator node called "Battery Low?" to check if the battery is low 
    and "Colliding?" to check if any obstacle is within minimum distance.
    Please refer to the py_trees documentation for more information on decorators.
    """
    ### YOUR CODE HERE ###


    idle = pt.behaviours.Running(name="Idle")

    """
    construct the behvior tree structure using the nodes and behaviors defined above
    """

    ### YOUR CODE HERE ###

    ## Example (only for navigation stack), integrate it with rest of the behavior tree
    root.add_child(navigation_stack)
    
    navigation_stack.add_children([mapping, localisation])

    mapping.add_child(gmapping)
    localisation.add_child(amcl)    

    return root

def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)
    # Initialising the node with name "behavior_tree"
    root = create_root()
    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
        )

    # setup the tree
    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=10)    
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()