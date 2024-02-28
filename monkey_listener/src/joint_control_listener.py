#!/usr/bin/env python3

# Description =============================================================================================
'''
This script was written during and for the bachelor thesis "Assembly and Programming of a
Robot Monkey to Study Imitation Learning in Marmosets". 

It is intended to be used with the robot monkey assembled during the mentioned thesis. 
Once all setup steps as described on the ReadMe on the source repository and in the appendix of the thesis have been completed, 
this script can be run on e.g a Raspberry Pi to control the mentioned robot monkey or a different robot.

Author: Jaú Gretler
E-Mail: gretleja@ethz.ch
Last changed: 13.6.23

Author: Joris Gentinetta
E-Mail: jorisg@ethz.ch
Last changed: 27.2.24

'''

# Imports ==================================================================================================

import argparse
import json
import os

from adafruit_servokit import ServoKit
from numpy import interp

kit = ServoKit(channels=16, frequency=333)

# Mapping from joint name to index in joint state message
MAPPING = {
    # "NH": 0,
    # "NF": 1,
    "RSF": 'R_Shoulder_Fro_Rot_Joint',
    "RSL": 'R_Shoulder_Lat_Joint',
    "RSH": 'R_Shoulder_Hor_Joint',
    "REB": 'R_Ellbow_Joint',
    "RW": 'R_Wrist_Joint',
    "RH": 'R_Hand_Joint',
    "LSF": 'L_Shoulder_Fro_Joint',
    "LSL": 'L_Shoulder_Lat_Joint',
    "LSH": 'L_Shoulder_Hor_Joint',
    "LEB": 'L_Ellbow_Joint',
    "LW": 'L_Wrist_Joint',
    "LH": 'L_Hand_Joint'
}
inverted_mapping = {value: key for key, value in MAPPING.items()}

current_file_path = os.path.abspath(__file__)
path_to_current_dir = os.path.dirname(current_file_path)


# Imports done =============================================================================================


# Joint class ==============================================================================================

# Class to store the duty cycles of the predefined states and who performs the mapping from duty cycle to [-1,1]
class Joint:

    def __init__(self, _name, _motor_pin, default_val=90, min_val=10, max_val=170, angmin=-1.571, angmax=1.571,
                 invert=False) -> None:
        # Init name, motor pin
        self.name = _name
        self.motor_pin = _motor_pin

        # min max turning range of the model joint in radians
        self.angMin = angmin
        self.angMax = angmax
        self.invert = invert

        # min max turning range of the actual joint in degrees [0, 180]
        self.min_val = min_val
        self.max_val = max_val
        self.default_val = default_val

        self.trajectory = [default_val]

        # Init motor
        self.servo = kit.servo[self.motor_pin]
        self.servo.set_pulse_width_range(900, 2200)
        self.servo.angle = self.default_val

    def add_to_trajectory(self, target_val, interpolation_steps=10, map=True):
        if map:
            intp_val = interp(target_val, [self.angMin, self.angMax],
                              [self.min_val, self.max_val])  # map model to robot
            if self.invert:
                intp_val = 180 - intp_val
        else:
            intp_val = target_val
        difference = intp_val - self.trajectory[-1]
        for i in range(1, interpolation_steps):
            self.trajectory.append(self.trajectory[-1] + difference / interpolation_steps)
        self.trajectory.append(float(intp_val))


# Joint class done =========================================================================================


# Body class ================================================================================================

# Structure to manage all joints of robots, contains all joints inside a dict
# Contains methods to set all joints to default position and to update joint state of physical robot according to incoming target joint state
class Body:
    def __init__(self) -> None:
        # Dict to store joint objects
        self.joints = {}

        # Left arm
        self.joints["LH"] = Joint("LH", 3, default_val=10, angmin=-1.4, angmax=1.4, min_val=165, max_val=170,
                                  invert=True)
        self.joints["LW"] = Joint("LW", 2, angmin=-1.4, angmax=1.4, min_val=10, max_val=170)
        self.joints["LEB"] = Joint("LEB", 1, angmin=-1.4, angmax=0, min_val=10, max_val=90, invert=True)
        self.joints["LSH"] = Joint("LSH", 4, angmin=-1.2, angmax=1.6, min_val=10, max_val=170, invert=True)
        self.joints["LSL"] = Joint("LSL", 0, angmin=-1.4, angmax=1.4, min_val=10, max_val=180, invert=True)
        self.joints["LSF"] = Joint("LSF", 5, angmin=-2.186, angmax=0.615, min_val=10, max_val=180)

        # Right arm
        self.joints["RH"] = Joint("RH", 9, default_val=170, angmin=-1.4, angmax=1.4, min_val=165, max_val=170)
        self.joints["RW"] = Joint("RW", 8, angmin=-1.4, angmax=1.4, min_val=10, max_val=180)
        self.joints["REB"] = Joint("REB", 7, angmin=0, angmax=1.4, min_val=90, max_val=170, invert=True)
        self.joints["RSH"] = Joint("RSH", 10, angmin=-1.6, angmax=1.2, min_val=10, max_val=170, invert=True)
        self.joints["RSL"] = Joint("RSL", 6, angmin=-1.4, angmax=1.4, min_val=10, max_val=180, invert=True)
        self.joints["RSF"] = Joint("RSF", 11, angmin=-2.186, angmax=0.615, min_val=10, max_val=180, invert=True)

        # Head
        self.joints["NH"] = Joint("NF", 12, angmin=-1.4, angmax=1.4, min_val=10, max_val=180)
        self.joints["NF"] = Joint("NH", 13, angmin=-1.4, angmax=1.4, min_val=10, max_val=180)


# Body class done ================================================================================================


def loadPlanFromJSON(file_name):
    path_name = path_to_current_dir + "/saved_plans/" + file_name + '.json'
    with open(path_name, 'rb') as f:
        jsonOjbect = json.load(f)
    return jsonOjbect


# Main ===========================================================================================================
if __name__ == '__main__':
    # Init node
    parser = argparse.ArgumentParser(description='Robot execution node')
    parser.add_argument('--interpolation_steps', type=int, default=10, required=False,
                        help='How many steps between waypoints')
    parser.add_argument('--filename', default=None, required=False, help='Name of the file to load')
    args = parser.parse_args()

    body = Body()
    jsonObject = loadPlanFromJSON(args.filename)
    jsonDict = json.loads(jsonObject)
    joint_names = jsonDict['joint_trajectory']['joint_names']
    for point in jsonDict['joint_trajectory']['points']:
        for joint_id, position in enumerate(point['positions']):
            joint_name = joint_names[joint_id]
            translated_name = inverted_mapping[joint_name]
            body.joints[translated_name].add_to_trajectory(position, int(args.interpolation_steps))
    for joint in body.joints.values():
        joint.add_to_trajectory(joint.default_val, int(args.interpolation_steps), map=False)

    max_len = 0
    for joint in body.joints.values():
        max_len = max(max_len, len(joint.trajectory))
    for joint in body.joints.values():
        if len(joint.trajectory) < max_len:
            joint.trajectory = joint.trajectory + [joint.trajectory[-1]] * (max_len - len(joint.trajectory))

    print('ready_for_execution')  # Signal to the GUI that the robot is ready to execute the plan
    execute = input()
    print(f'####################{execute}####################')
    if execute == 'start':
        print(f'starting trajectory with {len(body.joints["LH"].trajectory)} steps')
        for i in range(len(body.joints['LH'].trajectory)):
            for name, joint in body.joints.items():
                joint.servo.angle = joint.trajectory[i]
        print('finished_execution')  # Signal to the GUI that the robot has finished executing the plan

    print("Everything's cleaned up")

# Main done ============================================================================================
