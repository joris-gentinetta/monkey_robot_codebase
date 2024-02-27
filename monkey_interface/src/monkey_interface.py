#!/usr/bin/env python3

# Description =============================================================================================
'''
This script was written during and for the bachelor thesis "Assembly and Programming of a
Robot Monkey to Study Imitation Learning in Marmosets". 

It is intended to be used with the robot monkey assembled during the mentioned thesis. 
Once all setup steps as described on the ReadMe on the source repository and in the appendix of the thesis have been completed, 
this script can control the mentioned robot monkey or a different robot. 

This script interfaces the moveit move_group_python_interface, whose class reference can be found here:
https://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
(Note: At time of writing docs.ros.org is unreachable)

It was derived from the move_group_python_interface_tutorial which can be found here:
https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py


Author: Jaú Gretler
E-Mail: gretleja@ethz.ch
Last changed: 28.6.23

'''

# imports ==================================================================================================
import sys
import os
from os.path import join
import copy
import pathlib
import paramiko
import time
import rospy #pyright: ignore
import colorsys
import moveit_commander #pyright: ignore
import moveit_msgs.msg #pyright: ignore
from visualization_msgs.msg import InteractiveMarkerFeedback#pyright: ignore
import geometry_msgs #pyright: ignore
from  geometry_msgs.msg import Pose #pyright: ignore
from  geometry_msgs.msg import PoseArray #pyright: ignore
from visualization_msgs.msg import Marker,MarkerArray #pyright: ignore
from math import pi, dist, fabs, cos
from std_msgs.msg import String #pyright: ignore
from moveit_commander.conversions import pose_to_list #pyright: ignore
import json
import yaml #pyright: ignore
from rospy_message_converter import json_message_converter #pyright: ignore
from std_msgs.msg import String #pyright: ignore

try:
    from math import pi, TWO_PI, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    TWO_PI = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
EEF_STEP_SIZE = 10 #TODO TEST

current_file_path = os.path.abspath(__file__)
path_to_current_dir = os.path.dirname(current_file_path)
# imports done =============================================================================================

# Convenience functions ====================================================================================
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

# Convenience functions done ================================================================================

# MoveGroupInterface class ===============================================================================================================================
# This class serves as an interface for the moveit move_group node.
# This code was adapted from: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
class MoveGroupInterface(object): 
    "MoveGroupInterface"

    def __init__(self, planning_group_name):
        super(MoveGroupInterface, self).__init__()

        # Setup markerArray (for visualization of poses)
        self.markerArray = MarkerArray()
        self.mtestArray = MarkerArray()
        self.wp_m_id_counter = 2**10 # To disambiguate markers, each one gets an ID. For duplicate markers in MarkerArray only one marker is displayed
        self.im_m_id_counter = 0
        self.markerArrayPub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100) # Instantiate publisher for markerArray topic
        self.lrimpa_max_size = 3 # Max size of last recorded IM pose array
        self.lrimp_count = 0 # Counter for last recorded IM pose
        self.imp_buffer = []
        self.initial_wp_count = 1

        # Setup moveit_commander and dependent variables (robot, scene)
        moveit_commander.roscpp_initialize(sys.argv)
        # This node will appear in the ROS network under the name "move_group_interface". 
        # The anonymous flag renders the existence of multiple move_group nodes possible, which would otherwise cause a crash.
        rospy.init_node("move_group_interface", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        # Query planning group from user
        # target_planning_group_name = "monkey_left_arm" # Or input("Please specify planning group: ")
        
        # Setup move_group handle
        move_group = moveit_commander.MoveGroupCommander(planning_group_name)
        move_group.set_planning_time(15.0) # This sets a time limit for the planning of trajectories
        move_group.set_num_planning_attempts(30) # Number of planning attempts
        move_group.set_max_velocity_scaling_factor(1.0) # Scaler for execution of trajectories
        move_group.set_max_acceleration_scaling_factor(1.0)
        move_group.set_goal_position_tolerance(0.001) # Set position tolerance

        # Get eef link of target planning group
        eef_link = move_group.get_end_effector_link()
        print("End effector link: %s" % eef_link)

        # Get initial pose of eef
        self.eef_def_pose = move_group.get_current_pose().pose

        # Create a "DisplayTrajectory" ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Create a subscriber to the feedback of the IM in Rviz
        rospy.Subscriber("rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", InteractiveMarkerFeedback, self.intMarkerCallBack)
        
        # Variable to store last recorded position of IM in Rviz (received from Subscriber) 
        self.last_rec_im_pose = None
        # Array to store waypoints collected in Rviz self.gui
        self.wpoints = []

        # Indicator for whether we are appending to a loaded json pose array
        self.appending = False
                
        # Array to store loaded waypoints from json file
        self.loaded_json_wpoints = None
        # For rainbow coloring collected wpoints in marker creator this index is necessary
        self.loaded_json_wpoint_index = 0

        # Save some variables as member variables
        self.move_group = move_group
        self.robot = robot
        self.eef_link = eef_link


    # Callback function for interactive marker subscriber. Saves the last published IM pose and creates a marker for it.
    # The last <self.lrimpa_max_size> IM are saved, stored in a buffer and displayed as a visual aid to locate the last published IM pose
    def intMarkerCallBack(self,data):
        #print("len markers before callback body: ",len(self.markerArray.markers))
        #print("Entered int marker")
        # Extract pose
        p = data.pose
        # Create marker for it
        self.createMarker(p,"im_trace")
        # Append to pose buffer
        self.imp_buffer.append(p)
        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        if(self.lrimp_count > self.lrimpa_max_size):
            # Pop from markerArray
            self.mtestArray.markers.pop(0)
            # Pop from IM pose buffer
            self.imp_buffer.pop(0)
            # Renumber the marker IDs
            id = 0
            for m in self.mtestArray.markers:
                m.id = id
                id += 1
        # Publish the MarkerArray
        #self.markerArrayPub.publish(self.markerArray)
        self.markerArrayPub.publish(self.mtestArray)
        # Get idx of last recorded IM pose
        last_imp_idx = len(self.imp_buffer) - 1
        # Save last recorded IM pose
        self.last_rec_im_pose = self.imp_buffer[last_imp_idx]

        #print("len markers after callback body: ",len(self.markerArray.markers))

    # Set joint goal for eef of move_group, execute trajectory, check if target and final pose of eef are within tolerance [Adapter from tutorial]
    # def go_to_joint_goal(self):
    #     # Instantiate joint_state object
    #     joint_goal = self.move_group.get_current_joint_values()
    #     # Set joint states manually. ATTENTION: Must be in accordance to joint limits in URDF, otherwise an error will occur
    #     joint_goal[0] = 0
    #     joint_goal[1] = 0
    #     # Go to joint go state
    #     self.move_group.go(joint_goal, wait=True)
    #     # Calling ``stop()`` ensures that there is no residual movement
    #     self.move_group.stop()
    #     # Check if current and target joint state are within tolerance
    #     current_joints = self.move_group.get_current_joint_values()
    #     return all_close(joint_goal, current_joints, 0.01)

    # Create a (visual) marker for a pose 
    def createMarker(self,marker_pose,_ns):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp    = rospy.get_rostime()
        m.ns = _ns
        if _ns == "im_trace":
            m.id = self.im_m_id_counter
            self.im_m_id_counter += 1
        else:
            m.id = self.wp_m_id_counter
            self.wp_m_id_counter += 1
        m.type = 2 # sphere
        m.action = 0
        m.pose.position = copy.deepcopy(marker_pose.position)
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.005
        m.scale.y = 0.005
        m.scale.z = 0.005
        #color
        if _ns == "appended_waypoints":
            #orange
            r = 1.0
            g = 0.5
            b = 0.0
        elif _ns =="im_trace":
            #pink
            r = 1.0
            g = 0.0
            b = 1.0
        elif _ns == "collected_waypoint" or _ns == "hard_coded_waypoint": 
            # light blue
            r = 0.0
            g = 1.0
            b = 1.0
        elif _ns == "loaded_wpoint":
            if self.loaded_json_wpoints:
                r,g,b = colorsys.hsv_to_rgb(0.4+self.loaded_json_wpoint_index/100.0*2.5, 1.0, 1.0)
                self.loaded_json_wpoint_index += 1
        #print(self.m_id_counter)
        ## If the namespace of the input points identifies it als waypoint, color it in HSV color mode with the number of existing markers determining the hue
        #if _ns == "waypoint":
        #    r,g,b = colorsys.hsv_to_rgb(0.01+self.m_id_counter/360.0/4.0, 1.0, 1.0)
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0


        if _ns == "im_trace":
            self.mtestArray.markers.append(m)
        else:
            self.markerArray.markers.append(m)

        # To keep track of the buffer size for the IM poses we must count how many we created
        if _ns == "im_trace":
            self.lrimp_count += 1

    # Set pose goal for eef of move_group, execute trajectory, check if target and final pose of eef are within tolerance [Adapter from tutorial]
    # def go_to_pose_goal(self,tpose):
    #     # Extract target pose position
    #     pos = tpose.position
    #     pg_pos_arr = [pos.x,pos.y,pos.z]
    #     # Set pose goal
    #     self.move_group.set_position_target(pg_pos_arr,self.eef_link)
    #     # Move to pose goal, save success status. `go()` returns a boolean indicating whether the planning and execution was successful.
    #     success = self.move_group.go(wait=True)
    #     # Calling `stop()` ensures that there is no residual movement
    #     self.move_group.stop()
    #     self.move_group.clear_pose_targets()
    #     # For testing  tolerance
    #     current_pose_check = self.move_group.get_current_pose().pose
    #     return all_close(tpose, current_pose_check, 0.01)

    # Returns boolean indicating if there is valid trajectory from the eef def pos to a certain wpoint
    def isValid(self,wpoint):
        if not wpoint:
            print("No wpoint recorded. Have you moved the IM?")
            return None

        # Get pos of wpoint
        pos = wpoint.position
        # Create pos array
        pos_arr = [pos.x,pos.y,pos.z]
        # Set wpoint as pose goal
        self.move_group.set_position_target(pos_arr,self.eef_link)
        # Get a potential trajectory
        plan = self.move_group.plan()
        # The first element of the robot trajectory indicates whether the trajectory is valid
        return plan[0]


# Utility class mainly providing often used motion planning functionalities such as:
# - Planning, displaying and executing a single pose goal
# - Collecting waypoints in self.gui and computing a trajectory T for the eef of one planning group to go through
# - Saving, loading and editing of T
# This helper class could be integrated into the moveGroupInterface, this would however decrease code readability.
class Utils:
    def __init__(self, interface_left_arm, inteface_right_arm, interface_head, gui=None) -> None:
        # A handle to the moveGroupInteface is necessary for some actions.
        self.ifaceLA = interface_left_arm
        self.ifaceRA = inteface_right_arm
        # self.ifaceH = interface_head
        self.gui = gui

    # Let user collect an arbitrary number of waypoints in self.gui, return the collected waypoints

    def collect_wpoints(self):
        wcounter = self.initial_wp_count  # For the shell interaction a waypoint counter is needed. An initial_count != 0 means that we will append new poses to a poseArray loaded form a json file
        collecting_WP = True
        wp_ns = "collected_waypoint"
        self.col_posesLA = PoseArray()  # Create empty poseArray
        self.col_posesRA = PoseArray()
        # col_posesH = PoseArray()
        # If initial_wp_count is greater 1 this means we are not populating an empty poseArray, but instead appending new poses to an existing poseArray
        if self.initial_wp_count > 1:
            self.appending = True
            self.col_posesLA = self.ifaceLA.loaded_json_wpoints
            self.col_posesRA = self.ifaceRA.loaded_json_wpoints
            wp_ns = "appended_waypoints"
        # Collect as many poses as the user wants
        while collecting_WP == True:
            wpoint_valid = False
            # Try to get a valid wpoint
            while not wpoint_valid:
                if self.gui:
                    self.gui.addWaypointBtn.setText("Add Left Arm Waypoint")
                    self.gui.addWaypointBtn.setEnabled(True)
                    while not self.gui.add_waypoint:
                        pass
                    self.gui.addWaypointBtn.setEnabled(False)
                    self.gui.add_waypoint = False
                else:
                    print("Move left arm to wp ", wcounter)
                    input("To save waypoint press [Enter]")

                # Get the last recorded pose of the interactive marker
                # if self.iface.last_rec_im_pose:
                wLA = self.ifaceLA.last_rec_im_pose
                print(wLA)
                wpoint_valid = self.ifaceLA.isValid(wLA)
                if wpoint_valid:
                    # Plan a path
                    (planLA, suc_fracLA) = self.ifaceLA.move_group.compute_cartesian_path(self.col_posesLA.poses + [wLA], float(EEF_STEP_SIZE),
                                                                                 0.0)
                    if suc_fracLA == 1.0:
                        break
                    else:
                        out = 'Planning failed. Try again.'
                        if self.gui:
                            self.gui.outputText.append(out)
                        else:
                            print(out)
                else:
                    out = 'Invalid waypoint (outside robot range). Try again.'
                    if self.gui:
                        self.gui.outputText.append(out)
                    else:
                        print(out)

            wpoint_valid = False
            while not wpoint_valid:
                if self.gui:
                    self.gui.addWaypointBtn.setText("Add <b>Right Arm</b> Waypoint")
                    self.gui.addWaypointBtn.setEnabled(True)
                    while not self.gui.add_waypoint:
                        pass
                    self.gui.addWaypointBtn.setEnabled(False)
                    self.gui.add_waypoint = False
                else:
                    print("Move right arm to wp ", wcounter)
                    input("To save waypoint press [Enter]")
                # Get the last recorded pose of the interactive marker
                wRA = self.ifaceRA.last_rec_im_pose
                print(wRA)
                wpoint_valid = self.ifaceRA.isValid(wRA)
                if wpoint_valid:
                    # Plan a path
                    (planRA, suc_fracRA) = self.ifaceRA.move_group.compute_cartesian_path(self.col_posesRA.poses + [wRA], float(EEF_STEP_SIZE),
                                                                                 0.0)
                    if suc_fracRA == 1.0:
                        break
                    else:
                        out = 'Planning failed. Try again.'
                        if self.gui:
                            self.gui.outputText.append(out)
                        else:
                            print(out)
                else:
                    out = 'Invalid waypoint (outside robot range). Try again.'
                    if self.gui:
                        self.gui.outputText.append(out)
                    else:
                        print(out)

            # wpoint_valid = False
            # while not wpoint_valid:
            #     if self.gui:
            #         self.gui.addWaypointBtn.setText("Add <b>Head</b> Waypoint")
            #         self.gui.addWaypointBtn.setEnabled(True)
            #         while not self.gui.add_waypoint:
            #             pass
            #         self.gui.addWaypointBtn.setEnabled(False)
            #         self.gui.add_waypoint = False
            #     else:
            #         print("Move head to wp ", wcounter)
            #         input("To save waypoint press [Enter]")
            #     # Get the last recorded pose of the interactive marker
            #     # if self.iface.last_rec_im_pose:
            #     wH = self.ifaceH.last_rec_im_pose
            #     print(wH)
            #     wpoint_valid = self.ifaceH.isValid(wH)
            #     if wpoint_valid:
                    # Plan a path
                #     (planH, suc_fracH) = self.ifaceH.move_group.compute_cartesian_path(col_posesH.poses + [wH],
                #                                                                           float(EEF_STEP_SIZE),
                #                                                                           0.0)
                #     if suc_fracH == 1.0:
                #         break
                #     else:
                #         out = 'Planning failed. Try again.'
                #         if self.gui:
                #             self.gui.outputText.append(out)
                #         else:
               #               print(out)
                # else:
                #     out = 'Invalid waypoint (outside robot range). Try again.'
                #     if self.gui:
                #         self.gui.outputText.append(out)
                #     else:
                #        print(out)

            self.col_posesLA.poses.append(wLA)
            self.col_posesRA.poses.append(wRA)
            # col_posesH.poses.append(wH)
            # Append collected wpoint to wpoints stored in iface
            self.ifaceLA.wpoints.append(wLA)
            self.ifaceRA.wpoints.append(wRA)
            # self.ifaceH.wpoints.append(wH)
            # Create marker for this waypoint

            self.ifaceLA.createMarker(wLA, wp_ns)
            self.ifaceRA.createMarker(wRA, wp_ns)
            # self.ifaceH.createMarker(wH, wp_ns)

            wcounter += 1
            # Publish markers
            self.ifaceLA.markerArrayPub.publish(self.ifaceLA.markerArray)
            self.ifaceRA.markerArrayPub.publish(self.ifaceRA.markerArray)
            # self.ifaceH.markerArrayPub.publish(self.ifaceH.markerArray)
            # Query continuation of waypoint collection
            if self.gui:
                self.gui.saveBtn.setEnabled(True)
                self.gui.contBtn.setEnabled(True)
                while True:
                    if self.gui.save_waypoints:
                        filename = self.gui.fileNameEdit.text()
                        print("Waypoints saved to " + filename)
                        # self.gui.outputText.append("Waypoints saved to " + filename)
                        self.save_WP(self.col_posesLA, self.col_posesRA, filename)  # , col_posesH)
                        # collecting_WP = False
                        return self.col_posesLA, self.col_posesRA #, col_posesH
                    elif self.gui.cont_waypoints:
                        break
                self.gui.save_waypoints = False
                self.gui.cont_waypoints = False
                self.gui.saveBtn.setEnabled(False)
                self.gui.contBtn.setEnabled(False)
            else:
                cont = input("Do you want to continue collecting waypoints? [Enter | no]")
                if cont != "":
                    collecting_WP = False

        return self.col_posesLA, self.col_posesRA #, col_posesH
    # Query saving of poseArray from user, if desired query name of json file to save poseArray to. The file name without .json
    def save_WP(self,paLA, paRA, filename): #, paH):
        # Consctruct path name
        os.makedirs(path_to_current_dir + "/saved_waypoints", exist_ok=True)
        path_nameLA = path_to_current_dir + "/saved_waypoints/" + filename + '_LA.json'
        path_nameRA = path_to_current_dir + "/saved_waypoints/" + filename + '_RA.json'
        # Convert json poseArray to json
        json_pose_arrayLA = json_message_converter.convert_ros_message_to_json(paLA)
        json_pose_arrayRA = json_message_converter.convert_ros_message_to_json(paRA)
        # Dump json data into json file
        with open(path_nameLA, 'w+') as f:
            json.dump(json_pose_arrayLA, f)
        with open(path_nameRA, 'w+') as f:
            json.dump(json_pose_arrayRA, f)

    def save_plan(self, plan, filename):
        json_plan = json_message_converter.convert_ros_message_to_json(plan)

        # Consctruct path name
        os.makedirs(path_to_current_dir + "/saved_plans", exist_ok=True)
        path_name = path_to_current_dir + "/saved_plans/" + filename + '.json'
        # Dump json data into json file
        with open(path_name, 'w+') as f:
            json.dump(json_plan, f)

    def load_plan(self, filename):
        path_name = path_to_current_dir + "/saved_plans/" + filename + '.json'
        if os.path.exists(path_name):
            return True
        else:
            print("File does not exist, try again.")
            return False

            # with open(path_name, 'rb') as f:
            #     jsonOjbect = json.load(f)
            #     plan = json_message_converter.convert_json_to_ros_message('moveit_msgs/RobotTrajectory', jsonOjbect)
            #     return plan


    # Create three hardcoded waypoints
    # def pdExampleWaypoints(self):
    #     # Create waypoints based on current pose of end effector
    #     pa = PoseArray()
    #     current_pose = self.iface.move_group.get_current_pose().pose
    #     w0 = copy.deepcopy(current_pose)
    #     w1 = copy.deepcopy(w0)
    #     w1.position.z += 0.05
    #     w1.position.y += 0.06
    #     w1.position.x -= 0.02
    #     w2 = copy.deepcopy(w0)
    #     w2.position.z += 0.09
    #     w2.position.y += 0.07
    #     w2.position.x -= 0.0
    #     w3 = copy.deepcopy(w0)
    #     w3.position.z += 0.12
    #     w3.position.y += 0.07
    #     w3.position.x -= 0.02
    #     pa.poses = [w1,w2,w3]
    #     # Add markers for the created waypoints
    #     for w in pa.poses:
    #         self.iface.createMarker(w,'hard_coded_waypoint')
    #     # Publish the markerArray containing waypoint markers
    #     self.iface.markerArrayPub.publish(self.iface.markerArray)
    #     # Query execution
    #     self.queryCPP(pa)

    # Plan to a hardcoded pose goal, display the plan and query its execution
    # def pdExamplePoseGoal(self):
    #     # Get default pose of current end effector
    #     def_pose = self.iface.eef_def_pose
    #     # Create marker for default pose
    #     self.iface.createMarker(def_pose,"hard_coded_waypoint")
    #     # Create target pose
    #     pose_goal = copy.deepcopy(def_pose)
    #     # pose_goal.position.z += 0.05
    #     # pose_goal.position.y += 0.07
    #     # Create marker for target pose
    #     self.iface.createMarker(pose_goal,"hard_coded_waypoint")
    #     # Publish marker array containing def and target pose
    #     self.iface.markerArrayPub.publish(self.iface.markerArray)
    #     # Query planning and execution of single pose goal
    #     execute_pose_goal = input("Should the single pose goal be planned and executed? [yes | Enter]")
    #     if execute_pose_goal == "yes":
    #         self.iface.go_to_pose_goal(pose_goal)

    # Query if user wants to plan to a cartesian path. If so, plan it. Then, query for execution. If planning failed, exit.
    def CPP(self,col_paLA, col_paRA): #, col_paH):
        # Plan a path
        (planLA, suc_fracLA) = self.ifaceLA.move_group.compute_cartesian_path(col_paLA.poses, float(EEF_STEP_SIZE), 0.0) # last argument: jump_threshold -> not used
        (planRA, suc_fracRA) = self.ifaceRA.move_group.compute_cartesian_path(col_paRA.poses, float(EEF_STEP_SIZE), 0.0) # last argument: jump_threshold -> not used
        # (planH, suc_fracH) = self.ifaceH.move_group.compute_cartesian_path(col_paH.poses, float(EEF_STEP_SIZE), 0.0) # last argument: jump_threshold -> not used
        # Inform user of success fraction
        print(f"Success fraction LA: {suc_fracLA}, RA: {suc_fracRA}") #, H: {suc_fracH}")
        if suc_fracLA == 1.0 and suc_fracRA == 1.0: # and suc_fracH == 1.0:
            # Display the plan
            display_trajectoryLA = moveit_msgs.msg.DisplayTrajectory()
            display_trajectoryRA = moveit_msgs.msg.DisplayTrajectory()
            display_trajectoryLA.trajectory_start = self.ifaceLA.robot.get_current_state()
            display_trajectoryRA.trajectory_start = self.ifaceRA.robot.get_current_state()
            display_trajectoryLA.trajectory.append(planLA)
            display_trajectoryRA.trajectory.append(planRA)
            # Publish the plan
            self.ifaceLA.display_trajectory_publisher.publish(display_trajectoryLA)
            self.ifaceRA.display_trajectory_publisher.publish(display_trajectoryRA)

            combined_plan = self.join_plans(planLA, planRA)
            return combined_plan
        else:
            print("Planning failed")
            return None

    def join_plans(self, planL, planR):
        # Combine joint names from both plans
        print('planL: \n', planL)
        combined_plan = copy.deepcopy(planL)

        joint_names = planL.joint_trajectory.joint_names + planR.joint_trajectory.joint_names
        combined_plan.joint_trajectory.joint_names = joint_names

        # Assuming both plans have the same number of points and corresponding timestamps
        combined_plan.joint_trajectory.points = []
        for pointL, pointR in zip(planL.joint_trajectory.points, planR.joint_trajectory.points):
            point = copy.deepcopy(pointL)
            # Combine positions, velocities, accelerations, and effort
            point.positions = pointL.positions + pointR.positions
            point.velocities = pointL.velocities + pointR.velocities
            point.accelerations = pointL.accelerations + pointR.accelerations
            point.effort = pointL.effort + pointR.effort  # Assuming effort is relevant and should be concatenated

            combined_plan.joint_trajectory.points.append(point)
        print('combined_plan: \n', combined_plan)
        return combined_plan



    # Query the user for the name of json file containing a poseArray, then query saving of cart. path, then query execution of cart. path
    def loadWaypointsFromJSON(self, file_name):
        saved_files = os.listdir(path_to_current_dir + "/saved_waypoints")
        saved_files = list(set([f[:-8] for f in saved_files]))
        print("Please input name of json file to load. Options: ", saved_files)
        if file_name not in saved_files:
            print("File does not exist, try again. Options: ", saved_files)
            return False

        path_nameLA = path_to_current_dir + "/saved_waypoints/" + file_name + '_LA.json'
        path_nameRA = path_to_current_dir + "/saved_waypoints/" + file_name + '_RA.json'
        # path_nameH = path_to_current_dir + "/saved_waypoints/" + file_name + '_H.json'
        self.appending = True

        with open(path_nameLA, 'rb') as f:
            # Get poseArray data from json object
            jsonOjbect = json.load(f)
            col_posesLA = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseArray', jsonOjbect)
            # Save loaded poseArray in interface
            self.ifaceLA.loaded_json_wpoints = col_posesLA
            # Create marker for all poses in poseArray
            for pose in col_posesLA.poses:
                self.ifaceLA.createMarker(pose,'loaded_wpoint')
            self.ifaceLA.markerArrayPub.publish(self.ifaceLA.markerArray)
        with open(path_nameRA, 'rb') as f:
            jsonOjbect = json.load(f)
            col_posesRA = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseArray', jsonOjbect)
            self.ifaceRA.loaded_json_wpoints = col_posesRA
            for pose in col_posesRA.poses:
                self.ifaceRA.createMarker(pose,'loaded_wpoint')
            self.ifaceRA.markerArrayPub.publish(self.ifaceRA.markerArray)
        # with open(path_nameH, 'rb') as f:
        #     jsonOjbect = json.load(f)
        #     loaded_pose_array = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseArray', jsonOjbect)
        #     self.ifaceH.loaded_json_wpoints = loaded_pose_array
        #     for pose in loaded_pose_array.poses:
        #         self.ifaceH.createMarker
        #     self.ifaceH.markerArrayPub.publish(self.ifaceH.markerArray)
        if self.ifaceLA.loaded_json_wpoints == None:
            print("No waypoints were loaded!")
            return False
        else:
            print("Waypoints were loaded and are being displayed")
            return True





    # Query a valid 'mode' from the user, where mode is one of the scripts functionalities:
    # {single pose goal, hardcoded trajectory,  collecting waypoints, loading and editing waypoints, exit}
    def queryValidMode(self):
        print("")
        print("What do you want to do? : ")
        print("[1]  Collect waypoints")
        print("[2]  Load and edit saved waypoints")
        print("[3]  Load a saved plan and execute it")
        print("[4]  Exit")
        print("")

        mode = -1
        while True:
            mode_in = input("Select mode [ 1 | 2 | 3 | 4 ]: ")
            try:
                mode = int(mode_in)
                if mode not in (1, 2, 3, 4):
                    print("Unknown mode")
                else:
                    break
            except ValueError:
                print("Invalid input")
        return mode

    # Clear terminal and greet user
    def greet(self):
        os.system('clear') # Clear terminal
        print("")
        print("----------------------------------------------------------")
        print("Monkey Interface")
        print("----------------------------------------------------------")



    def interact_with_monkey_listener(self, ssh_host='10.42.0.2', ssh_port=22, ssh_user='rm', ssh_key_path='/home/robot-user/.ssh/id_rsa',
                                    remote_script_path='/home/rm/ws_moveit/src/monkey_listener/src/joint_control_listener.py', interpolation_steps=10):
        """
        Interact with a remote script using SSH key for authentication.

        :param ssh_host: String, the hostname or IP of the remote machine.
        :param ssh_port: Integer, the port number to connect to on the remote machine.
        :param ssh_user: String, the username for authentication.
        :param ssh_key_path: String, the path to the SSH private key file.
        :param remote_script_path: String, the path to the remote script that requires interaction.
        """
        filename = self.gui.fileNameEdit.text()
        # Initialize SSH client
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # Load SSH key
        # ssh_key = paramiko.RSAKey.from_private_key_file(ssh_key_path)

        # Connect to the remote machine
        ssh.connect(hostname=ssh_host, port=ssh_port, username=ssh_user)#, pkey=ssh_key)

        # Start an interactive shell session
        shell = ssh.invoke_shell()

        def wait_until_ready(wait_time):
            while True:
                time.sleep(wait_time)
                if shell.recv_ready():
                    output = shell.recv(4096).decode()
                    print(output, '\n')
                    if 'ready_for_execution' in output:
                        break

        # copy the file to the remote machine
        sftp = ssh.open_sftp()
        sftp.put(join(path_to_current_dir, 'saved_plans', f'{filename}.json'), join(os.path.dirname(remote_script_path),
                                                                                    'saved_plans', f'{filename}.json'))
        sftp.close()

        # Start the remote script
        print(shell.send(f'python3 {remote_script_path} --from_json True --interpolation_steps {interpolation_steps} --filename {filename}\n'))
        wait_until_ready(2)
        self.gui.startBtn.setEnabled(True)

        # Interact with the script
        while self.gui.start == False:
            pass
        self.gui.start = False

        print(shell.send('start\n'))

        time.sleep(1)
        output = shell.recv(4096).decode()
        print(output, '\n')

        # Close the SSH connection
        ssh.close()




def main():
    pass
    # try:
    #     # TODO  adapt CLI to GUI
    #     # Create interface to Moveit:RobotCommander (move-group python interface)
    #     interface_left_arm = MoveGroupInterface('monkey_left_arm')
    #     interface_right_arm = MoveGroupInterface('monkey_right_arm')
    #     interface_head = MoveGroupInterface('head')
    #
    #     # Create helper class
    #     helper = Utils(interface_left_arm, interface_right_arm, interface_head)
    #
    #     # Greet user
    #     helper.greet()
    #
    #     # Query user for a valid mode
    #     mode = helper.queryValidMode()
    #
    #     if mode == 1: # Let user collect waypoints in self.gui, query save and finally query execution
    #
    #         # Collect waypoints in self.gui
    #         # collected_pose_arrayLA, collected_pose_arrayRA, collected_pose_arrayH = helper.collect_wpoints(1)
    #         collected_pose_arrayLA, collected_pose_arrayRA = helper.collect_wpoints(1)
    #         # Query save
    #         helper.querySave(collected_pose_arrayLA, collected_pose_arrayRA)  # , collected_pose_arrayH)
    #         # Make sure that all user set waypoints are being displayed
    #         interface_left_arm.markerArrayPub.publish(interface_left_arm.markerArray)
    #         interface_right_arm.markerArrayPub.publish(interface_right_arm.markerArray)
    #         interface_head.markerArrayPub.publish(interface_head.markerArray)
    #
    #         # Query decision to plan car. path
    #         # helper.queryCPP(collected_pose_arrayLA, collected_pose_arrayRA, collected_pose_arrayH)
    #         plan = helper.queryCPP(collected_pose_arrayLA, collected_pose_arrayRA)
    #         if plan:
    #             # Query decision to execute cart. plan
    #             exec_dec = input("Do you want to execute the cart. plan? [Enter for yes, any key for no]")
    #             if exec_dec == "":
    #                 helper.ifaceLA.move_group.execute(plan,
    #                                                 wait=True)  # Waits until feedback from execution is received
    #             ans = input("Do you want to save the plan? [yes | Enter]")
    #             if ans != "":
    #                 filename = input("Please specify an (unused) filename without file type: ")
    #                 helper.save_plan(plan, filename)
    #     elif mode == 2:  # Load waypoints from json, potentially edit them
    #         # Query json file name, load poseArray, query appending new poses
    #         helper.loadWaypointsFromJSON()
    #     elif mode == 3:  # Load a saved plan and execute it
    #         # Load a saved plan and execute it
    #         helper.interact_with_monkey_listener()
    #
    #     elif mode == 4: # Exit
    #         pass
    #
    #     else:
    #         print("Unrecognized mode")
    #
    #     # Before exiting
    #     print("Exiting")
    #
    #
    # except rospy.ROSInterruptException:
    #     return
    # except KeyboardInterrupt:
    #     return


if __name__ == "__main__":
    main()
