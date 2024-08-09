import py_trees
import rclpy
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from rclpy.node import Node
from robot_interfaces.msg import UrcCustomPoint, UrcCustomPath
import numpy as np
from time import sleep
import random

# Need GUI to implement client to connect to service.
from robot_interfaces.srv import GUIWaypointPath

# Need to implement GPS to XYZ client
# Need to implement MAVROS topics
# Need to implement Tf Tree topics

# ArUco Detection Service
from robot_interfaces.srv import ObjectPosition

# Search Algorithm Service
from robot_interfaces.srv import GeneratePointsAround
from robot_interfaces.srv import GeneratePoints

# PID Planner services topics
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from std_srvs.srv import SetBool
from robot_interfaces.srv import SetPIDPlannerPrecision
from pid_controller_lib import PID_Controller


class GetWaypointLocation(Behaviour):
    def __init__(self, waypoint_manager, path, name="GetWaypointLocation"):
        super(GetWaypointLocation, self).__init__(name)
        self.waypoint_manager = waypoint_manager
        self.path = None # Will change this to None when connecting the GUI.
        self.gui_node = None
        self.gui_service_client_server = None
        
    def setup(self):
        self.gui_node = Node("get_waypoint_path")
        self.gui_service_client_server = self.gui_node.create_service(GUIWaypointPath, "get_waypoint_path", self.get_waypoint_path_callback)
        self.logger.debug(f"GetWaypointLocation::setup {self.name}")

    # Ensure the way point path is correct type.
    def get_waypoint_path_callback(self, request, response):
        if request.path is None:
            self.logger.error(f"Incoming request: {request.path} isn't valid")
            return response
        
        self.path = request.path.points
        self.logger.info(f"Incoming request: {request.path} was received!")
        return response

    def initialise(self):
        self.logger.debug(f"GetWaypointLocation::initialise {self.name}")

    def update(self):
       if self.path is not None:
        waypoint = self.path[self.waypoint_manager.get("index")]
        self.waypoint_manager.curr_waypoint = waypoint
        self.waypoint_manager.waypoint_type = waypoint.location_label
        self.waypoint_manager.waypoint_location = waypoint.point.point
        
        print(f"The waypoint manager variables {self.waypoint_manager}")
        self.logger.debug(f"GetWaypointLocation::update {self.name}")
        return Status.SUCCESS
       
       else:
           self.logger.debug("Waiting for waypoint path...")
           self.feedback_message = 'Waiting for waypoint path...'
           return Status.RUNNING
    
    def terminate(self, new_status):
        self.logger.debug(f"GetWaypointLocation::terminate {self.name} to {new_status}")

class MoveToWaypoint(Behaviour):
    def __init__(self, waypoint_manager, name="MoveToWaypoint"):
        super(MoveToWaypoint, self).__init__(name=name)
        self.waypoint_manager = waypoint_manager
        
        # Create the publisher and client.
        self.pub_current_waypoint_node = None
        self.pub_current_waypoint = None
        self.arrived_at_waypoint_node = None
        self.arrived_at_waypoint_service_client = None
        

    def setup(self):
        # Setup the publisher and client for the PID to start checking for arrival.
        self.pub_current_waypoint_node = Node("current_waypoint_node")
        self.pub_current_waypoint = self.pub_current_waypoint_node.create_publisher(Point, "current_waypoint_topic", 10)
        self.arrived_at_waypoint_node = Node("arrived_at_waypoint_node")
        self.arrived_at_waypoint_service_client = self.arrived_at_waypoint_node.create_client(SetBool, "enabled_service")
        if self.arrived_at_waypoint_service_client.wait_for_service(1.0):
            self.logger.info("Arrived at waypoint service is available")
        else:
            self.logger.warning("Arrived at waypoint service is not available.")
        self.logger.debug(f"MoveToWaypoint::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"MoveToWaypoint::initialise {self.name}")   

    def update(self):
        # Publish the current waypoint location for the PID
        curr_waypoint_msg = Point()
        curr_waypoint_msg.x = self.waypoint_manager.waypoint_location.x
        curr_waypoint_msg.y = self.waypoint_manager.waypoint_location.y
        curr_waypoint_msg.z = self.waypoint_manager.waypoint_location.z
        self.pub_current_waypoint.publish(curr_waypoint_msg)
        
        self.arrived_at_waypoint_req = SetBool.Request()
        self.arrived_at_waypoint_future = self.arrived_at_waypoint_service_client.call_async(self.arrived_at_waypoint_req)
        rclpy.spin_until_future_complete(self.arrived_at_waypoint_future)
        self.arrived_at_waypoint_response = self.arrived_at_waypoint_future.result()

        # Will need to change arrived to the response. 
        self.logger.debug(f"MoveToWaypoint::update {self.name}")
        arrived = random.choice([True, False])
        if not arrived:
            self.feedback_message = "Moving towards waypoint..."
            return Status.RUNNING
        else:
            self.feedback_message = "Arrived"
            self.waypoint_manager.index += 1
            return Status.SUCCESS
        
    def terminate(self, new_status):
        
        self.logger.debug(f"MoveToWaypoint::terminate {self.name} to {new_status}")
        
class DetectObjects(Behaviour):
    def __init__(self, waypoint_manager, name="DetectObjects"):
        super(DetectObjects, self).__init__(name=name)
        
        # This client will be responsible for search points.
        self.search_points = py_trees.blackboard.Client("search_points_manager")
        self.search_points.register_key(key="search_point", access=py_trees.common.Access.READ)
        self.search_points.register_key(key="search_point", access=py_trees.common.Access.WRITE)
        self.search_points.register_key(key="index", access=py_trees.common.Access.READ)
        self.search_points.register_key(key="index", access=py_trees.common.Access.WRITE)

        self.waypoint_manager = waypoint_manager
        # Will need the publisher to be updated 
        
        # Arrival
        self.arrived_at_waypoint_node = None
        self.arrived_at_waypoint_service_client = None

        # Search algorithm connection
        self.get_points_node = None
        self.get_points_around_node = None
        self.get_points_service_client = None
        self.get_points_around_service_client=None



    def setup(self):
        # Ready for testing on actually working. 
        # self.get_points_node = Node("get_points_node")
        # self.get_points_around_node = Node("get_points_around_node")
        # self.get_points_service_client = self.get_points_node.create_client(GeneratePoints, "GeneratePoints")
        # self.get_points_around_service_client = self.get_points_around_node.create_client(GeneratePointsAround,"GeneratePointsAround")
        # if self.get_points_service_client.wait_for_service(1.0):
        #     self.logger.debug("Service get points is available")
        # else:
        #     self.logger.warning("Service get points is not available")
        
        # if self.get_points_around_service_client.wait_for_service(1.0):
        #     self.logger.debug("Service get points around is available")
        # else:
        #     self.logger.warning("Service get points around is not available.")

        self.logger.debug(f"DetectObjects::setup {self.name}")
    
    def initialise(self):
        self.feedback_message = None
        self.waypoint_type = self.waypoint_manager.get("waypoint_type")
        self.logger.debug(f"DetectObjects::initialise {self.name}")

    # Implement logic for detecting the objects here
    def update(self):
        if self.waypoint_type != "object":
            self.feedback_message = "Waypoint is not type object."
            return Status.FAILURE
         
        # self.get_points_req = GeneratePoints.Request()
        # self.get_points_around_req = GeneratePointsAround.Request()
        # # self.get_points_around_req.waypoint = 
        
        # self.get_points_future = self.get_points_service_client.call_async(self.get_points_req)
        # self.get_points_around_future = self.get_points_around_service_client.call_async(self.get_points_around_req)
        # rclpy.spin_until_future_complete(self.get_points_node, self.get_points_future)
        # rclpy.spin_until_future_complete(self.get_points_around_node, self.get_points_around_future)
        # response_get_points = self.get_points_future.result()
        # response_get_points_around = self.get_points_around_future.result()

        # Can confirm it's connecting just need to send it a UrcCustomPoint

        # if self.get_points_future.result() is not None:
        #     print(response_get_points)
        #     return Status.SUCCESS
        
        # if self.get_points_around_future.result is not None:
        #     print(response_get_points_around)
        #     return Status.SUCCESS
        
        # else:
        #     print("Error trying to grab get points")
        #     return Status.FAILURE
        
        object_detected = random.choice([True, False])
        if self.waypoint_type != "object":
            self.feedback_message = "Waypoint is not type object."
            return Status.FAILURE
        elif not object_detected:
            self.feedback_message = "Searching for object..."
            return Status.RUNNING
        else:
            self.feedback_message = "Success! Object is found!"
            # self.waypoint_manager.flag_for_next = 1
            return Status.SUCCESS

    def terminate(self, new_status):
        self.feedback_message = None
        self.logger.debug(f"DetectObjects::terminate {self.name} to {new_status}")
        
class DetectArUcoMarker(Behaviour):
    def __init__(self, waypoint_manager, name="DetectArUcoMarker"):
        super(DetectArUcoMarker, self).__init__(name=name)
        self.waypoint_manager = waypoint_manager
        
        # Aruco client
        self.aruco_localization_node = None
        self.aruco_localization_service_client = None

        # This client will be responsible for search points.
        self.search_points = py_trees.blackboard.Client("search_points_manager")
        self.search_points.register_key(key="search_point", access=py_trees.common.Access.READ)
        self.search_points.register_key(key="search_point", access=py_trees.common.Access.WRITE)
        self.search_points.register_key(key="index", access=py_trees.common.Access.READ)
        self.search_points.register_key(key="index", access=py_trees.common.Access.WRITE)

        # Creation of the search algorithm
        self.get_points_node = None
        self.get_points_around_node = None
        self.get_points_service_client = None
        self.get_points_around_service_client=None

        # Arrival
        self.arrived_at_waypoint_node = None
        self.arrived_at_waypoint_service_client = None


    def setup(self):
        # Ready for testing on actually working. 
        # self.get_points_node = Node("get_points_node")
        # self.get_points_around_node = Node("get_points_around_node")
        # self.get_points_service_client = self.get_points_node.create_client(GeneratePoints, "GeneratePoints")
        # self.get_points_around_service_client = self.get_points_around_node.create_client(GeneratePointsAround,"GeneratePointsAround")
        # if self.get_points_service_client.wait_for_service(1.0):
        #     self.logger.debug("Service get points is available")
        # else:
        #     self.logger.warning("Service get points is not available")
        
        # if self.get_points_around_service_client.wait_for_service(1.0):
        #     self.logger.debug("Service get points around is available")
        # else:
        #     self.logger.warning("Service get points around is not available.")

        # self.aruco_localization_node = Node("aruco_localization_client_node")
        # self.aruco_localization_service_client = self.aruco_localization_node.create_client(ObjectPosition, "get_aruco_positions")
        # if self.aruco_localization_service_client.wait_for_service(1.0):
        #     self.logger.debug("Service is available.")
        # else:
        #     self.logger.warning("Service is not available")
        self.logger.debug(f"DetectArUcoMarker::setup {self.name}")
    
    def initialise(self):
        self.feedback_message = None
        self.waypoint_type = self.waypoint_manager.get("waypoint_type")
        self.logger.debug(f"DetectArUcoMarker::initialise {self.name}")

    # Implement logic for detecting an ArUco Marker here
    def update(self):
        # self.aruco_local_req = ObjectPosition.Request()
        # self.aruco_local_future = self.aruco_localization_service_client.call_async(self.aruco_local_req)
        # rclpy.spin_until_future_complete(self.aruco_localization_node, self.aruco_local_future)
        # response = self.aruco_local_future.result()
        # if self.aruco_local_future.result() is not None:
        #     print(response)
        #     return Status.SUCCESS
        # else:
        #     print("Error didn't get the result.")
        #     return Status.FAILURE
        # self.get_points_req = GeneratePoints.Request()
        # self.get_points_around_req = GeneratePointsAround.Request()
        # # self.get_points_around_req.waypoint = 
        
        # self.get_points_future = self.get_points_service_client.call_async(self.get_points_req)
        # self.get_points_around_future = self.get_points_around_service_client.call_async(self.get_points_around_req)
        # rclpy.spin_until_future_complete(self.get_points_node, self.get_points_future)
        # rclpy.spin_until_future_complete(self.get_points_around_node, self.get_points_around_future)
        # response_get_points = self.get_points_future.result()
        # response_get_points_around = self.get_points_around_future.result()

        # Can confirm it's connecting just need to send it a UrcCustomPoint

        # if self.get_points_future.result() is not None:
        #     print(response_get_points)
        #     return Status.SUCCESS
        
        # if self.get_points_around_future.result is not None:
        #     print(response_get_points_around)
        #     return Status.SUCCESS
        
        # else:
        #     print("Error trying to grab get points")
        #     return Status.FAILURE

        marker_detected = random.choice([True, False])
        # Implement the correct logic for marker detection.
        if self.waypoint_type != "aruco":
            self.feedback_message = "Waypoint is not type aruco."
            return Status.FAILURE
        if not marker_detected:
            self.feedback_message = "Searching for ArUco Marker..."
            return Status.RUNNING
        else:
            self.feedback_message = "Success! ArUco Marker detected!"
            self.waypoint_manager.flag_for_next = 1
            return Status.SUCCESS
        
    def terminate(self, new_status):
        self.feedback_message = None
        self.logger.debug(f"DetectArUcoMarker::terminate {self.name} to {new_status}")

class IsIntermediate(Behaviour):
    def __init__(self, waypoint_manager, name="IsIntermediate"):
        super(IsIntermediate, self).__init__(name=name)
        self.waypoint_manager = waypoint_manager

    def setup(self):
        self.logger.debug(f"IsIntermediate::setup {self.name}")
    
    def initialise(self):
        self.feedback_message = None
        self.waypoint_type = self.waypoint_manager.get("waypoint_type")
        self.logger.debug(f"IsIntermediate::initialise {self.name}")

    def update(self):
        if self.waypoint_type != "intermediate":
            self.feedback_message="Waypoint type is not intermediate."
            return Status.FAILURE
        else:
            self.feedback_message="Waypoint is intermediate, move to next waypoint." 
            return Status.SUCCESS

    def terminate(self, new_status):
        self.feedback_message = None
        self.logger.debug(f"IsIntermediate::terminate {self.name} to {new_status}")    


class ArrivalSeq(py_trees.behaviour.Behaviour):
    def __init__(self, waypoint_manager, name="ArrivalSeq"):
        super(ArrivalSeq, self).__init__(name=name)
        self.waypoint_manager = waypoint_manager

        # This client will be responsible for search points.
        self.search_points = py_trees.blackboard.Client("search_points_manager")
        self.search_points.register_key(key="search_point", access=py_trees.common.Access.READ)
        self.search_points.register_key(key="search_point", access=py_trees.common.Access.WRITE)
        self.search_points.register_key(key="index", access=py_trees.common.Access.READ)
        self.search_points.register_key(key="index", access=py_trees.common.Access.WRITE)
        
         # Creation of the search algorithm
        self.get_points_node = None
        self.get_points_around_node = None
        self.get_points_service_client = None
        self.get_points_around_service_client=None
        
        # Arrival
        self.arrived_at_waypoint_node = None
        self.arrived_at_waypoint_service_client = None
    
    def setup(self):
        # Ready for testing on actually working. 
        # self.get_points_node = Node("get_points_node")
        # self.get_points_around_node = Node("get_points_around_node")
        # self.get_points_service_client = self.get_points_node.create_client(GeneratePoints, "GeneratePoints")
        # self.get_points_around_service_client = self.get_points_around_node.create_client(GeneratePointsAround,"GeneratePointsAround")
        # if self.get_points_service_client.wait_for_service(1.0):
        #     self.logger.debug("Service get points is available")
        # else:
        #     self.logger.warning("Service get points is not available")
        
        # if self.get_points_around_service_client.wait_for_service(1.0):
        #     self.logger.debug("Service get points around is available")
        # else:
        #     self.logger.warning("Service get points around is not available.")
        self.logger.debug(f"ArrivalSeq::setup {self.name}")
    
    def initialise(self):
        self.feedback_message=None
        self.waypoint_type = self.waypoint_manager.get("waypoint_type")
        self.logger.debug(f"ArrivalSeq::initalise {self.name}")

    def update(self):
        # Will need to place a check here to ensure the rover isn't already at the goal point.
        # self.get_points_req = GeneratePoints.Request()
        # self.get_points_around_req = GeneratePointsAround.Request()
        # # self.get_points_around_req.waypoint = 
        
        # self.get_points_future = self.get_points_service_client.call_async(self.get_points_req)
        # self.get_points_around_future = self.get_points_around_service_client.call_async(self.get_points_around_req)
        # rclpy.spin_until_future_complete(self.get_points_node, self.get_points_future)
        # rclpy.spin_until_future_complete(self.get_points_around_node, self.get_points_around_future)
        # response_get_points = self.get_points_future.result()
        # response_get_points_around = self.get_points_around_future.result()

        # Can confirm it's connecting just need to send it a UrcCustomPoint

        # if self.get_points_future.result() is not None:
        #     print(response_get_points)
        #     return Status.SUCCESS
        
        # if self.get_points_around_future.result is not None:
        #     print(response_get_points_around)
        #     return Status.SUCCESS
        
        # else:
        #     print("Error trying to grab get points")
        #     return Status.FAILURE
        if self.waypoint_type != "goalpoint":
            self.feedback_message = "Waypoint is not type goalpoint"
            return Status.FAILURE
        else:
            self.feedback_message = "Waypoint is a goalpoint!"
            # self.waypoint_manager.flag_for_next = 1
            return Status.SUCCESS
        
    def terminate(self, new_status):
        self.feedback_message = None
        self.logger.debug(f"ArrivalSeq::terminate {self.name} to {new_status}")

class IdleBehavior(Behaviour):
    def __init__(self, waypoint_manager, name="IdleBehavior"):
        super(IdleBehavior, self).__init__(name=name)
        self.waypoint_manager = waypoint_manager
        # Going to need something for the user input and idle until the rover is ready to continue.
    
    def setup(self):
        self.logger.debug(f"IdleBehavior::setup {self.name}")

    def initialise(self):
        self.idle_flag = False
        self.logger.debug(f"IdleBehavior::initialise {self.name}")

    def update(self):
        # Add the service for user input when it lands to this behavior. Can't directly call for user input in the tree. 
        sleep(1)
        # ready_for_next = self.waypoint_manager.get("flag_for_next") 
        self.idle_flag = random.choice([True, False])
        # if ready_for_next == 0:
        #     self.feedback_message = "Not ready for next waypoint."
        #     return Status.FAILURE      
        if not self.idle_flag:
            self.feedback_message = "Waiting for user input..."
            return Status.RUNNING
        else:
            self.feedback_message = "Success! Moving forward..."
            # self.waypoint_manager.flag_for_next = 0
            return Status.SUCCESS
        
    def terminate(self, new_status):
        self.idle_flag = False
        self.feedback_message = None
        self.logger.debug(f"ArrivalSeq::terminate {self.name} to {new_status}")     
         