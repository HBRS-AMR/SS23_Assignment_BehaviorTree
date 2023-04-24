import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
import std_msgs.msg as msg
import rospy
import sys
from behaviors import *

def create_root():
    """
    Method to construct the behavior tree
    
    The "battery_check" behavior tree writes the voltage of the battery on 'blackboard' of behavior tree
    for each initialisation of traversal of ticks. If the battery level is lower than a threshold, the
    robot will execute certain behavior. Here the robot rotation is used to indicate low level of the battery
    """
    ## define nodes and behaviors

    # define root node
    root = pt.composites.Parallel("root")

    # define node which includes data on topics to be written to blackboard

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
    Main function that initiates behavior tree construction
    """
    # Initialising the node with name "behavior_tree"
    rospy.init_node("behavior_tree")
    root = create_root()
    behaviour_tree = ptr.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    # printing status of nodes in ASCII format
    tick_printer = lambda t: pt.display.print_ascii_tree(t.root, show_status=True)
    behaviour_tree.tick_tock(sleep_ms=3000, post_tick_handler=tick_printer)    

if __name__ == '__main__':
    main()    
