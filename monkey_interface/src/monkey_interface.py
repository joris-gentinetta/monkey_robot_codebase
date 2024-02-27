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

Author: Joris Gentinetta
E-Mail: jorisg@ethz.ch
Last changed: 27.2.24

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
from moveit_commander.conversions import pose_to_list #pyright: ignore
import json
from rospy_message_converter import json_message_converter #pyright: ignore

try:
    from math import pi, TWO_PI, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    TWO_PI = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
EEF_STEP_SIZE = 10 #TODO TEST
USE_HEAD = False

current_file_path = os.path.abspath(__file__)
path_to_current_dir = os.path.dirname(current_file_path)
# imports done =============================================================================================


# MoveGroupInterface class ===============================================================================================================================
# This class serves as an interface for the moveit move_group node.
# This code was adapted from: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
class MoveGroupInterface(object): 
    "MoveGroupInterface"

    def __init__(self, planning_group_name):
        super(MoveGroupInterface, self).__init__()

        # Setup moveit_commander and dependent variables (robot, scene)
        moveit_commander.roscpp_initialize(sys.argv)
        # This node will appear in the ROS network under the name "move_group_interface". 
        # The anonymous flag renders the existence of multiple move_group nodes possible, which would otherwise cause a crash.
        rospy.init_node("move_group_interface", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        # Query planning group from user
        # target_planning_group_name = "monkey_left_arm" # Or input("Please specify planning group: ")
        
        # Setup move_group handle
        self.move_group = moveit_commander.MoveGroupCommander(planning_group_name)
        self.move_group.set_planning_time(15.0) # This sets a time limit for the planning of trajectories
        self.move_group.set_num_planning_attempts(100) # Number of planning attempts
        self.move_group.set_max_velocity_scaling_factor(1.0) # Scaler for execution of trajectories
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.move_group.set_goal_position_tolerance(0.001) # Set position tolerance

        # Get eef link of target planning group
        self.eef_link = self.move_group.get_end_effector_link()
        print("End effector link: %s" % self.eef_link)

        # Get initial pose of eef
        self.eef_def_pose = self.move_group.get_current_pose().pose

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
        # Array to store waypoints collected in Rviz gui
        self.wpoints = []
                
        # Array to store loaded waypoints from json file
        self.loaded_json_wpoints = None




    # Callback function for interactive marker subscriber. Saves the last published IM pose
    def intMarkerCallBack(self,data):
        # Extract pose
        p = data.pose
        self.last_rec_im_pose = p


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
        self.move_group.set_position_target(pos_arr, self.eef_link)
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
    def __init__(self, gui=None) -> None:
        # A handle to the moveGroupInteface is necessary for some actions.
        self.ifaceLA = MoveGroupInterface('monkey_left_arm')
        self.ifaceRA = MoveGroupInterface('monkey_right_arm')
        if USE_HEAD:
            self.ifaceH =  MoveGroupInterface('head')
        self.gui = gui
        self.initial_wp_count = 1

    def collect_wpoint(self, extremity):
        wpoint_valid = False
        if extremity == 'Left':
            iface = self.ifaceLA
            col_poses = self.col_posesLA
        elif extremity == 'Right':
            iface = self.ifaceRA
            col_poses = self.col_posesRA
        elif extremity == 'Head':
            iface = self.ifaceH
            col_poses = self.col_posesH
        else:
            print("Invalid extremity")
            return

        # Try to get a valid wpoint
        while not wpoint_valid:
            self.gui.addWaypointBtn.setText(f"Add {extremity} Waypoint")
            self.gui.addWaypointBtn.setEnabled(True)
            while not self.gui.add_waypoint:
                pass
            self.gui.addWaypointBtn.setEnabled(False)
            self.gui.add_waypoint = False

            # Get the last recorded pose of the interactive marker
            # if self.iface.last_rec_im_pose:
            w = iface.last_rec_im_pose
            wpoint_valid = iface.isValid(w)
            if wpoint_valid:
                # Plan a path
                (plan, suc_frac) = iface.move_group.compute_cartesian_path(col_poses.poses + [w],
                                                                           float(EEF_STEP_SIZE),
                                                                           0.0)
                if suc_frac == 1.0:
                    break
                else:
                    wpoint_valid = False
                    out = 'Planning failed. Try again.'
                    self.gui.outputText.append(out)
            else:
                out = 'Invalid waypoint (outside robot range). Try again.'
                self.gui.outputText.append(out)

        col_poses.poses.append(w)
        iface.wpoints.append(w)


    # Let user collect an arbitrary number of waypoints in self.gui, return the collected waypoints
    def collect_wpoints(self):
        if self.initial_wp_count > 1:
            self.col_posesLA = self.ifaceLA.loaded_json_wpoints
            print("Loaded waypoints from file")
            print(self.col_posesLA)
        else:
            self.col_posesLA = PoseArray()

        if self.initial_wp_count > 1:
            self.col_posesRA = self.ifaceRA.loaded_json_wpoints
        else:
            self.col_posesRA = PoseArray()

        if USE_HEAD:
            if self.initial_wp_count > 1:
                self.col_posesH = self.ifaceH.loaded_json_wpoints
            else:
                self.col_posesH = PoseArray()

        n = self.initial_wp_count
        # Collect as many poses as the user wants
        while True:
            self.collect_wpoint('Left')
            self.gui.outputText.append(f"Waypoint {n} collected for left arm")
            self.collect_wpoint('Right')
            self.gui.outputText.append(f"Waypoint {n} collected for right arm")
            if USE_HEAD:
                self.collect_wpoint('Head')
                self.gui.outputText.append(f"Waypoint {n} collected for head")

            self.gui.saveBtn.setEnabled(True)
            self.gui.contBtn.setEnabled(True)
            while True:
                if self.gui.save_waypoints:
                    filename = self.gui.fileNameEdit.text()
                    self.gui.outputText.append("Waypoints saved to " + filename)
                    if USE_HEAD:
                        self.save_WP(self.col_posesLA, self.col_posesRA, self.col_posesH, filename)
                    else:
                        self.save_WP(self.col_posesLA, self.col_posesRA, None, filename)

                    return
                elif self.gui.cont_waypoints:
                    break
            self.gui.save_waypoints = False
            self.gui.cont_waypoints = False
            self.gui.saveBtn.setEnabled(False)
            self.gui.contBtn.setEnabled(False)
            n += 1


    # Query saving of poseArray from user, if desired query name of json file to save poseArray to. The file name without .json
    def save_WP(self,paLA, paRA, paH, filename):
        # Consctruct path name
        os.makedirs(path_to_current_dir + "/saved_waypoints", exist_ok=True)

        path_nameLA = path_to_current_dir + "/saved_waypoints/" + filename + '_LA.json'
        json_pose_arrayLA = json_message_converter.convert_ros_message_to_json(paLA)
        with open(path_nameLA, 'w+') as f:
            json.dump(json_pose_arrayLA, f)

        path_nameRA = path_to_current_dir + "/saved_waypoints/" + filename + '_RA.json'
        json_pose_arrayRA = json_message_converter.convert_ros_message_to_json(paRA)
        with open(path_nameRA, 'w+') as f:
            json.dump(json_pose_arrayRA, f)

        if USE_HEAD:
            path_nameH = path_to_current_dir + "/saved_waypoints/" + filename + '_HE.json'
            json_pose_arrayH = json_message_converter.convert_ros_message_to_json(paH)
            with open(path_nameH, 'w+') as f:
                json.dump(json_pose_arrayH, f)

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


    # Query if user wants to plan to a cartesian path. If so, plan it. Then, query for execution. If planning failed, exit.
    def CPP(self,col_paLA, col_paRA, col_paH):
        # Plan a path
        (planLA, suc_fracLA) = self.ifaceLA.move_group.compute_cartesian_path(col_paLA.poses, float(EEF_STEP_SIZE), 0.0) # last argument: jump_threshold -> not used
        (planRA, suc_fracRA) = self.ifaceRA.move_group.compute_cartesian_path(col_paRA.poses, float(EEF_STEP_SIZE), 0.0) # last argument: jump_threshold -> not used
        # Inform user of success fraction
        if USE_HEAD:
            (planH, suc_fracH) = self.ifaceH.move_group.compute_cartesian_path(col_paH.poses, float(EEF_STEP_SIZE),
                                                                               0.0)  # last argument: jump_threshold -> not used
            condition = suc_fracLA == 1.0 and suc_fracRA == 1.0 and suc_fracH == 1.0
        else:
            planH = None
            condition = suc_fracLA == 1.0 and suc_fracRA == 1.0

        if condition:
            combined_plan = self.join_plans(planLA, planRA, planH)
            return combined_plan
        else:
            return None

    def join_plans(self, planL, planR, planH):
        # Combine joint names from both plans
        combined_plan = copy.deepcopy(planL)

        joint_names = planL.joint_trajectory.joint_names + planR.joint_trajectory.joint_names
        if USE_HEAD:
            joint_names += planH.joint_trajectory.joint_names
        combined_plan.joint_trajectory.joint_names = joint_names

        combined_plan.joint_trajectory.points = []
        if USE_HEAD:
            for pointL, pointR, pointH in zip(planL.joint_trajectory.points, planR.joint_trajectory.points, planH.joint_trajectory.points):
                point = copy.deepcopy(pointL)
                # Combine positions, velocities, accelerations, and effort
                point.positions = pointL.positions + pointR.positions + pointH.positions
                point.velocities = pointL.velocities + pointR.velocities + pointH.velocities
                point.accelerations = pointL.accelerations + pointR.accelerations + pointH.accelerations
                point.effort = pointL.effort + pointR.effort + pointH.effort

                combined_plan.joint_trajectory.points.append(point)
        else:
            for pointL, pointR in zip(planL.joint_trajectory.points, planR.joint_trajectory.points):
                point = copy.deepcopy(pointL)
                # Combine positions, velocities, accelerations, and effort
                point.positions = pointL.positions + pointR.positions
                point.velocities = pointL.velocities + pointR.velocities
                point.accelerations = pointL.accelerations + pointR.accelerations
                point.effort = pointL.effort + pointR.effort

                combined_plan.joint_trajectory.points.append(point)
        return combined_plan


    # Query the user for the name of json file containing a poseArray, then query saving of cart. path, then query execution of cart. path
    def loadWaypointsFromJSON(self, file_name):
        saved_files = os.listdir(path_to_current_dir + "/saved_waypoints")
        saved_files = list(set([f[:-8] for f in saved_files]))
        if file_name not in saved_files:
            self.gui.outputText.append("File does not exist, try again. Options: " + str(saved_files))
            return False

        path_nameLA = path_to_current_dir + "/saved_waypoints/" + file_name + '_LA.json'
        with open(path_nameLA, 'rb') as f:
            jsonOjbect = json.load(f)
            self.ifaceLA.loaded_json_wpoints = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseArray', jsonOjbect)

        path_nameRA = path_to_current_dir + "/saved_waypoints/" + file_name + '_RA.json'
        with open(path_nameRA, 'rb') as f:
            jsonOjbect = json.load(f)
            self.ifaceRA.loaded_json_wpoints = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseArray', jsonOjbect)

        if USE_HEAD:
            path_nameH = path_to_current_dir + "/saved_waypoints/" + file_name + '_HE.json'
            with open(path_nameH, 'rb') as f:
                jsonOjbect = json.load(f)
                self.ifaceH.loaded_json_wpoints = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseArray', jsonOjbect)




        if self.ifaceLA.loaded_json_wpoints == None:
            self.gui.outputText.append("Error loading waypoints from file")
            return False
        else:
            self.initial_wp_count = len(self.ifaceLA.loaded_json_wpoints.poses) + 1
            self.gui.outputText.append("Waypoints loaded from " + file_name)
            return True







    def interact_with_monkey_listener(self, ssh_host='10.42.0.2', ssh_port=22, ssh_user='rm', ssh_key_path='/home/robot-user/.ssh/id_rsa',
                                    remote_script_path='/home/rm/ws_moveit/src/monkey_listener/src/joint_control_listener.py'):
        """
        Interact with a remote script using SSH key for authentication.

        :param ssh_host: String, the hostname or IP of the remote machine.
        :param ssh_port: Integer, the port number to connect to on the remote machine.
        :param ssh_user: String, the username for authentication.
        :param ssh_key_path: String, the path to the SSH private key file.
        :param remote_script_path: String, the path to the remote script that requires interaction.
        """
        filename = self.gui.fileNameEdit.text()
        interpolation_steps = int(self.gui.interpEdit.text())
        # Initialize SSH client
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # Load SSH key
        # ssh_key = paramiko.RSAKey.from_private_key_file(ssh_key_path)

        # Connect to the remote machine
        ssh.connect(hostname=ssh_host, port=ssh_port, username=ssh_user)#, pkey=ssh_key)

        # Start an interactive shell session
        shell = ssh.invoke_shell()

        def wait_until_ready(wait_time, clue):
            while True:
                time.sleep(wait_time)
                if shell.recv_ready():
                    output = shell.recv(4096).decode()
                    print(output, '\n')
                    if clue in output:
                        break

        # copy the file to the remote machine
        sftp = ssh.open_sftp()
        sftp.put(join(path_to_current_dir, 'saved_plans', f'{filename}.json'), join(os.path.dirname(remote_script_path),
                                                                                    'saved_plans', f'{filename}.json'))
        sftp.close()

        # Start the remote script
        print(shell.send(f'python3 {remote_script_path} --from_json True --interpolation_steps {interpolation_steps} --filename {filename}\n'))
        wait_until_ready(2, 'ready_for_execution')
        self.gui.startBtn.setEnabled(True)
        self.gui.outputText.append("Ready to start movement")

        # Interact with the script
        while self.gui.start == False:
            pass
        self.gui.start = False

        print(shell.send('start\n'))

        wait_until_ready(2, 'finished_execution')

        # Close the SSH connection
        ssh.close()

if __name__ == "__main__":
    # main()
    pass