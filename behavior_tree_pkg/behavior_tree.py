import random
import rclpy
from builtin_interfaces.msg import Time
from time import sleep
import py_trees
from py_trees import logging as log_tree
from robot_interfaces.msg import UrcCustomPath, UrcCustomPoint
from behavior_tree_pkg.actions_conditions import GetWaypointLocation, MoveToWaypoint, DetectArUcoMarker, DetectObjects, IsIntermediate, ArrivalSeq, IdleBehavior

def create_behavior_tree(path):
    # Create the blackboard to use for the tree
    waypoint_manager = py_trees.blackboard.Client(name="waypoint_manager")
    waypoint_manager.register_key(key="curr_waypoint", access=py_trees.common.Access.READ)
    waypoint_manager.register_key(key="curr_waypoint", access=py_trees.common.Access.WRITE)
    waypoint_manager.register_key(key="waypoint_type", access=py_trees.common.Access.READ)
    waypoint_manager.register_key(key="waypoint_type", access=py_trees.common.Access.WRITE)
    waypoint_manager.register_key(key="waypoint_location", access=py_trees.common.Access.WRITE)
    waypoint_manager.register_key(key="waypoint_location", access=py_trees.common.Access.READ)
    waypoint_manager.register_key(key="index", access=py_trees.common.Access.WRITE)
    waypoint_manager.register_key(key="index", access=py_trees.common.Access.READ)
    waypoint_manager.register_key(key="flag_for_next", access=py_trees.common.Access.WRITE)
    waypoint_manager.register_key(key="flag_for_next", access=py_trees.common.Access.READ)
    waypoint_manager.index = 0
      
    # Initialize the root of the behavior tree
    root = py_trees.composites.Sequence("Root", memory=True)
    
    # Get the waypoint the rover needs to move towards
    get_waypoint_location = GetWaypointLocation(waypoint_manager, path, name="GetWaypointLocation")
    root.add_child(get_waypoint_location)

    # # Move to the waypoint
    # move_to_waypoint = MoveToWaypoint(waypoint_manager, name="MoveToWaypoint")
    # root.add_child(move_to_waypoint)

    # # Create a selector for detemining waypoint type to each behavior
    # selector_node = py_trees.composites.Selector("waypoint_selector", memory=True)
    
    # # Create the intermediate check first.
    # is_intermediate = IsIntermediate(waypoint_manager, name="IsIntermediate")
    # selector_node.add_child(is_intermediate)
    
    # # Create the different sequences to perform on anything that isn't the intermediate point
    # detect_aruco_sequence = py_trees.composites.Sequence("DetectArUcoSequence", memory=True)
    # detect_object_sequence = py_trees.composites.Sequence("DetectObjectSequence", memory=True)
    # goal_arrival_sequence = py_trees.composites.Sequence("GoalArrivalSequence", memory=True)
    
    # # Create the idle behavior
    # idle_goal_behavior = IdleBehavior(waypoint_manager, name="IdleBehavior")
    # idle_aruco_behavior = IdleBehavior(waypoint_manager, name="IdleBehavior")
    # idle_object_behavior = IdleBehavior(waypoint_manager,name="IdleBehavior")
    
    
    # # Create the detect behaviors and arrival sequence
    # detect_aruco_marker = DetectArUcoMarker(waypoint_manager, name="DetectArUcoMarker")
    # detect_object = DetectObjects(waypoint_manager, name="DetectObjects")
    # arrival_sequence = ArrivalSeq(waypoint_manager, name="ArrivalSeq")
    
    # # Add the detect children to detect aruco sequence
    # detect_aruco_sequence.add_child(detect_aruco_marker)
    # detect_aruco_sequence.add_child(idle_goal_behavior)  
    
    # # Add the children to the detect object sequence
    # detect_object_sequence.add_child(detect_object)
    # detect_object_sequence.add_child(idle_aruco_behavior)
    
    # # Add the children to the goal arrival sequence
    # goal_arrival_sequence.add_child(arrival_sequence)
    # goal_arrival_sequence.add_child(idle_object_behavior)
    
    # # Add the children to the selctor node.
    # # CURRENT ORDER: 1) Intermediate 2) Goal point sequence 3) Detect ArUco sequence 4) Detect object sequence
    # selector_node.add_child(goal_arrival_sequence)
    # selector_node.add_child(detect_aruco_sequence)
    # selector_node.add_child(detect_object_sequence) 

    # root.add_child(selector_node)
    
    # # idle = py_trees.behaviours.Running(name="idle")
    # # root.add_child(idle)

    return root

def print_tree(tree):
    print(py_trees.display.unicode_tree(root=tree.root, show_status=True))


def main():
    rclpy.init()

    log_tree.level = log_tree.Level.INFO

    waypoints_path = UrcCustomPath()

    for i in range(10):
        waypoint = UrcCustomPoint()
        waypoint.point.point.x = random.uniform(0.0, 10.0)
        waypoint.point.point.y = random.uniform(0.0, 10.0)
        waypoint.point.point.z = random.uniform(0.0, 10.0)
        waypoint.point.header.frame_id = 'global_origin_frame'
        waypoint.location_label = random.choice(['intermediate', 'goalpoint', 'object', 'aruco'])

        
        waypoints_path.points.append(waypoint)

    # Use this when ready for different choices to appear.
    
    # Testing purposes only to see how I can access this data.
    # print(waypoints_path)
    # print(waypoints_path.points[0].point.point)

    # waypoints_type_list = []
    # for i in range(10):
    #     waypoints_type_list.append(random.choice(["object"]))


    # Create the behavior tree
    behavior_tree = py_trees.trees.BehaviourTree(root=create_behavior_tree(waypoints_path.points))
    behavior_tree.setup()
    try:
        behavior_tree.tick_tock(
            period_ms=1000,
            number_of_iterations= py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=print_tree,
        )
    except KeyboardInterrupt:
        behavior_tree.interrupt()
    

    rclpy.shutdown()

if __name__ == "__main__":
    main()