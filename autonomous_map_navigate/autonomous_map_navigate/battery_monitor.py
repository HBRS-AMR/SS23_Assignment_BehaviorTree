#!/usr/bin/env python3

import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
import rclpy
import sys
from autonomous_map_navigate.behaviors import *

def create_root():
    """
    Method to structure the behavior tree to monitor battery status and start rotation if battery is low
    
    The "battery_monitor" behavior tree writes the voltage of the battery on 'blackboard' of behavior tree. 
    If the battery level is lower than a threshold, the robot will execute certain behavior. Here the robot 
    rotation is used to indicate low level of the battery
    """
    ## define nodes and behaviors

    # defining root node
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
    Using the battery_status2bb class, create a node called "Battery2BB" which subscribes to the topic "/battery_voltage"
    """
    ### YOUR CODE HERE ###

    """
    Using the rotate class, create a node called "rotate_platform"
    """
    ### YOUR CODE HERE ###

    """
    Read the 'battery_low_warning' from the blackboard and set a decorator node called "Battery Low?" to check if the battery is low.
    Please refer to the py_trees documentation for more information on decorators.
    """
    ### YOUR CODE HERE ###


    idle = pt.behaviours.Running(name="Idle")
    

    """
    construct the behvior tree structure using the nodes and behaviors defined above
    """

    ### YOUR CODE HERE ###

    return root

def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)

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