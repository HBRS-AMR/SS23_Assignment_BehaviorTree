import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
from sensor_msgs.msg import LaserScan
import rospy
import sys
from behaviors import *

def create_root():
    """
    Method to construct the behavior tree

    The "collison_avoidance" behavior tree adds new features to the "check_battery" behavior tree which 
    is to avois collison with obstacles. Whenever the robot is about to collide with an object, the robot will
    automatically stop, overriding the input commands. The robot can be controlled either by joystick, 
    where the command is published on the '/joy' topic or by command that is published on '/cmd_vel' topic.
    The laser scan data will be stored in blackboard by reading '/scan_filtered' topic. When an obstacle 
    is detected, the 'stop_motion' behavior will be executed.
    """

    ## define nodes and behaviors

    # define root node
    root = pt.composites.Parallel("root")

    # define node which includes data on topics (/scan_filtered, /mileage) to be written to blackboard

    ## YOUR CODE HERE ##



    # define nodes corresponding to different behaviors, along with the node 
    # defining the strategy of selection of these behaviours

    ## YOUR CODE HERE ##




    # construct the behvior tree structure using the nodes defined above

    ## YOUR CODE HERE ##
    


    return root

def shutdown(behaviour_tree):
    """
    This method will be called on termination of the behavior tree
    """
    rospy.loginfo("[SHUTDOWN] shutting down the behavior_tree")
    behaviour_tree.interrupt()

def main():
    """
    Main function initiates behavior tree construction
    """
    # Initialising the node with name "behavior_tree"
    rospy.init_node("behavior_tree")
    root = create_root()
    behaviour_tree = ptr.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    tick_printer = lambda t: pt.display.print_ascii_tree(t.root, show_status=True)
    behaviour_tree.tick_tock(sleep_ms=10, post_tick_handler=tick_printer)    

if __name__ == '__main__':
    main()    
