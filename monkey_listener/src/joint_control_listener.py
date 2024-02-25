#!/usr/bin/env python3

# Description =============================================================================================
'''
This script was written during and for the bachelor thesis "Assembly and Programming of a
Robot Monkey to Study Imitation Learning in Marmosets". 

It is intended to be used with the robot monkey assembled during the mentioned thesis. 
Once all setup steps as described on the ReadMe on the source repository and in the appendix of the thesis have been completed, 
this script can be run on e.g a Raspberry Pi to control the mentioned robot monkey or a different robot. 

This script launches a ROS node subscribed to the 'joint_states' topic which continuously listens to incoming target joint state data.
It then uses the newest data in 'joint_states' to update the servo motors duty cycles accordingly. 

The servo motor installed in the robot monkey during the thesis is the KST A12-T, whose rotational range can be covered by setting the duty cycle of a PWM signal to [4.5,10.5] %. 
The rotational range of the mechanical joint is 180° 
However, effects such as friction and varying tension make it such that the servos can often not pull hard enough at the threads to achieve the full ROM.
Thus, each Joint object in this script has its own so called predefined states, namely {min,middle,max,def}, which contain the duty cycle for a particular joint (and servo) for a particular state.
The states are not based on the duty cycle range based on the data sheet, but rather on the duty cycles which, when applied to the servos, 
lead to them e.g reaching their lower/upper rotational limit as permitted by the mentioned effects. Similarly, 
the middle and def value are the duty cycles for two other useful states, to which the user can then to refer to on a higher level. 
The 'def' state refers to the duty cycle the servo must have, such that the joint corresponding to the servo is at an angle such that the planning group of which the joint is part of (e.g left arm) is in its resting position.

To reduce jitter, the PiGPIOFactory from gpiozero is used. 
As a consequence, the final value written to the servo motor has to be in [-1,1].
To make use of the predefined states (especially min,max) mapping functions were created which take the following things into account:
- potentially inversed attachment of threads to servo wheel
- different angle ranges of joints (comes from URDF, which in turn comes from the URDF exporter from SolidWorks)

Author: Jaú Gretler
E-Mail: gretleja@ethz.ch
Last changed: 13.6.23

'''

# Imports ==================================================================================================

import time
import os
import json
import pathlib
import argparse
import rospy
from numpy import interp
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16, frequency=333)

# Mapping from joint name to index in joint state message
MAPPING = {
    "NH": 0,
    "NF": 1,
    "RSF": 2,
    "RSL": 3,
    "RSH": 4,
    "REB": 5,
    "RW": 6,
    "RH": 7,
    "LSF": 8,
    "LSL": 9,
    "LSH": 10,
    "LEB": 11,
    "LW": 12,
    "LH": 13
}
inverted_mapping = {value: key for key, value in MAPPING.items()}
# Imports done =============================================================================================


# Joint class ==============================================================================================

# Class to store the duty cycles of the predefined states and who performs the mapping from duty cycle to [-1,1]
class Joint:

    def __init__(self, _name, _motor_pin, default_val=90, min_val=10, max_val=170, angmin=-1.571, angmax=1.571, invert=False, from_json=False) -> None:
        # Init name, motor pin
        self.name = _name
        self.motor_pin = _motor_pin

        # Map min,middle,max,default value from full duty cycle range [4.5,10.5] to [-1,1] (necessary for PiGPIOFactory())
        self.min_val = min_val
        self.max_val = max_val
        self.default_val = default_val

        # Init motor (by using the factory pattern from the gpiozero lib we can significantly reduce the jitter)
        self.servo = kit.servo[self.motor_pin]
        self.servo.set_pulse_width_range(900, 2200)
        self.servo.angle = self.default_val

        # Init model angle (per default set all to [-pi/half,pi/half])
        self.angMin = angmin
        self.angMax = angmax
        self.invert = invert

        self.from_json = from_json
        self.trajectory = [default_val]


    # Set joint servo to a value in [-1,1].
    # The input target_val is mapped to [-1,1], where the mapping also depends on the attachment of the threads to the servo.
    def set_to_itp_val(self, target_val):
        # Calculate the actual value to be written to the servo depending on which mapping type the servo has
        intp_val = interp(target_val, [self.angMin, self.angMax], [self.min_val, self.max_val])
        if self.invert:
            # Attachment inversion
            intp_val = 180 - intp_val

        # Write interpolated value to servo
        prev_val = self.servo.angle
        time1 = time.time()
        self.servo.angle = float(intp_val)
        time2 = time.time()
        if abs(prev_val - intp_val) > 0.1:
            # Log interpolated value and original target value
            intp_val = '%.2f' % intp_val
            target_val = '%.3f' % target_val
            # target_val_deg = int(math.degrees(target_val))
            rospy.loginfo("Setting %s to [%s radians] -> [itp %s] ", self.name, target_val, intp_val)
    def add_to_trajectory(self, target_val, interpolation_steps=10):
        intp_val = interp(target_val, [self.angMin, self.angMax], [self.min_val, self.max_val])
        if self.invert:
            intp_val = 180 - intp_val
        difference = intp_val - self.trajectory[-1]
        for i in range(1, interpolation_steps):
            self.trajectory.append(self.trajectory[-1] + difference/interpolation_steps)
        self.trajectory.append(float(intp_val))
# Joint class done =========================================================================================


# Body class ================================================================================================

# Structure to manage all joints of robots, contains all joints inside a dict
# Contains methods to set all joints to default position and to update joint state of physical robot according to incoming target joint state
class Body:
    def __init__(self, from_json) -> None:

        # Dict to store joint objects
        self.joints = {}

        # Init joints [name,pin,default,min,middle,max]

        # Left arm
        self.joints["LH"] = Joint("LH",3, default_val=10, angmin=-1.4, angmax=1.4, min_val=165, max_val=170, invert=True, from_json=from_json)
        self.joints["LW"] = Joint("LW", 2, angmin=-1.4, angmax=1.4, min_val=10, max_val=170, from_json=from_json)
        self.joints["LEB"] = Joint("LEB", 1, angmin=-1.4, angmax=0, min_val=10, max_val=90, invert=True, from_json=from_json)
        self.joints["LSH"] = Joint("LSH", 4, angmin=-1.2, angmax=1.6, min_val=10, max_val=170, invert=True, from_json=from_json)
        self.joints["LSL"] = Joint("LSL", 0, angmin=-1.4, angmax=1.4, min_val=10, max_val=180, invert=True, from_json=from_json)
        self.joints["LSF"] = Joint("LSF", 5, angmin=-2.186, angmax=0.615, min_val=10, max_val=180, from_json=from_json)

        # Right arm
        self.joints["RH"] = Joint("RH",9, default_val=170, angmin=-1.4, angmax=1.4, min_val=165, max_val=170, from_json=from_json)
        self.joints["RW"] = Joint("RW", 8, angmin=-1.4, angmax=1.4, min_val=10, max_val=180, from_json=from_json)
        self.joints["REB"] = Joint("REB", 7, angmin=0, angmax=1.4, min_val=90, max_val=170, invert=True, from_json=from_json)
        self.joints["RSH"] = Joint("RSH", 10, angmin=-1.6, angmax=1.2, min_val=10, max_val=170, invert=True, from_json=from_json)
        self.joints["RSL"] = Joint("RSL", 6, angmin=-1.4, angmax=1.4, min_val=10, max_val=180, invert=True, from_json=from_json)
        self.joints["RSF"] = Joint("RSF", 11, angmin=-2.186, angmax=0.615, min_val=10, max_val=180, invert=True, from_json=from_json)

        # Head
        self.joints["NH"] = Joint("NF", 12, angmin=-1.4, angmax=1.4, min_val=10, max_val=180, from_json=from_json)
        self.joints["NF"] = Joint("NH", 13, angmin=-1.4, angmax=1.4, min_val=10, max_val=180, from_json=from_json)

    # Set all servos to default position
    def allToDef(self):
        for k, j in self.joints.items():
            j.angle = j.default_val
            rospy.loginfo("Setting %s to [itp %s]", j.name, j.default_val)
        rospy.sleep(1.0)

    # Update joints according to incoming joint target joint states
    def updateJoints(self):
        time_update = time.time()
        # This list contains all active joints. If a joint name is removed, it will not get updated. Note that LSH/RSH are currently not contained
        # selected_joints = ["LSL"]
        for joint_name, target_value in target_joint_positions.items():
            # if joint_name in selected_joints:
            J = self.joints[joint_name]
            # if J.name in working_joints:
            J.set_to_itp_val(target_value)
        time_update2 = time.time()
        print("Time to update all joints: ", time_update2 - time_update)


# Body class done ================================================================================================

def loadPlanFromJSON(file_name):
    path_to_current_dir = str(pathlib.Path().resolve()) # The path gets loaded from the moveit workspace top folder
    saved_files = os.listdir(path_to_current_dir + "/saved_plans")
    saved_files = list(set([f.split('.')[0] for f in saved_files]))
    path_name = path_to_current_dir + "/saved_plans/" + file_name + '.json'

    with open(path_name, 'rb') as f:
        # Get poseArray data from json object
        jsonOjbect = json.load(f)
    return jsonOjbect

# Main ===========================================================================================================

# Callback function which is executed when the joint-state listener receives new data
def callback(JointState):
    time_callback = time.time()
    # Copy incoming joint state target data into storage variable
    for key, value in MAPPING.items():
        target_joint_positions[key] = JointState.position[value]


    # Update joints according to target joint states
    body.updateJoints()
    time_callback_2 = time.time()
    print('time to callback: ', time_callback_2-time_callback)

if __name__ == '__main__':
    # Init node
    parser = argparse.ArgumentParser(description='Robot execution node')
    parser.add_argument('--from_json', default=False, required=False, help='Load plan from json file')
    parser.add_argument('--interpolation_steps', default=10, required=False, help='How many steps between waypoints')
    parser.add_argument('--filename', default=None, required=False, help='Name of the file to load')
    args = parser.parse_args()

    if args.from_json:
        body = Body(args.from_json)
        jsonObject = loadPlanFromJSON(args.filename)
        for point in jsonObject.points:
            for joint_id, position in enumerate(point.positions):
                joint_name = inverted_mapping[joint_id]
                body.joints[joint_name].add_to_trajectory(position, args.interpolation_steps)
        execute = input("Press enter to start the trajectory")
        if execute == '':
            for i in range(len(body.joints['LH'].trajectory)):
                for joint in body.joints.values():
                    joint.servo.angle = joint.trajectory[i]

    else:
        rospy.init_node("monkey_listener")
        rospy.loginfo("Monkey listener node has started")

        # Create body object which manages joints
        body = Body(args.from_json)

        # Set all joint to def positions
        body.allToDef()

        # For storing target joint positions which subscriber receives
        target_joint_positions = {}

        # Create subscriber to joint-state topic
        rospy.Subscriber("/joint_states", JointState, callback)

        # Spin() keeps python from exiting until this node is stopped
        rospy.spin()

    print("Everything's cleaned up")

# Main done ============================================================================================
