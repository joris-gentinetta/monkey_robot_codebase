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
import rospy
from numpy import interp
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16, frequency=333)

# Imports done =============================================================================================


# Joint class ==============================================================================================

# Class to store the duty cycles of the predefined states and who performs the mapping from duty cycle to [-1,1]
class Joint:

    def __init__(self, _name, _motor_pin, default_val=90, min_val=10, max_val=170, invert=False) -> None:
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
        self.angMin = -1.571
        self.angMax = 1.571
        self.invert = invert

    # Set joint servo to one of 4 pre defined states {0,0.5,1,def}
    def setState(self, _target_state):
        # Value to be written to servo
        next_val = 0
        if (_target_state == "def"):
            next_val = self.default_val
        elif (_target_state == "0"):
            next_val = self.min_val
        elif (_target_state == "0.5"):
            self.servo.angle = self.default_val
        elif (_target_state == "1"):
            next_val = self.max_val
        else:
            rospy.loginfo("Unknown target state")
            return
        # Log which value was set to which motor
        rospy.loginfo("Setting %s to [itp %s]", self.name, next_val)
        # Write value to motor
        self.servo.angle = next_val

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
        self.servo.angle = float(intp_val)
        if abs(prev_val - intp_val) > 0.1:
            # Log interpolated value and original target value
            intp_val = '%.2f' % intp_val
            target_val = '%.3f' % target_val
            # target_val_deg = int(math.degrees(target_val))
            rospy.loginfo("Setting %s to [%s radians] -> [itp %s] ", self.name, target_val, intp_val)

    # # Sweep through predefined states
    # def sweep(self):
    #     delay = 1.0
    #     self.setState("0")
    #     rospy.sleep(delay)
    #     self.setState("0.5")
    #     rospy.sleep(delay)
    #     self.setState("1")
    #     rospy.sleep(delay)
    #     self.setState("def")
    #     rospy.sleep(delay)


# Joint class done =========================================================================================


# Body class ================================================================================================

# Structure to manage all joints of robots, contains all joints inside a dict
# Contains methods to set all joints to default position and to update joint state of physical robot according to incoming target joint state
class Body:
    def __init__(self) -> None:

        # Dict to store joint objects
        self.joints = {}

        # Init joints [name,pin,default,min,middle,max]

        # Left arm
        self.joints["LH"] = Joint("LH",3, invert=True)
        self.joints["LW"] = Joint("LW", 2)
        self.joints["LEB"] = Joint("LEB", 1, invert=True)
        self.joints["LSH"] = Joint("LSH", 4)
        self.joints["LSL"] = Joint("LSL", 0, invert=True)
        self.joints["LSF"] = Joint("LSF", 5)

        # Right arm
        self.joints["RH"] = Joint("RH",9)
        self.joints["RW"] = Joint("RW", 8)
        self.joints["REB"] = Joint("REB", 7)
        self.joints["RSH"] = Joint("RSH", 10)
        self.joints["RSL"] = Joint("RSL", 6)
        self.joints["RSF"] = Joint("RSF", 11)

        # Head
        self.joints["NH"] = Joint("NF", 12)
        self.joints["NF"] = Joint("NH", 13)




        # # Set special angles and mapTypes
        #
        # self.joints["LEB"].angMin = 0
        # self.joints["LEB"].angMax = 1.571
        #
        # self.joints["LSH"].angMin = -1.571
        # self.joints["LSH"].angMax = -0.4955
        #
        # self.joints["RSF"].angMin = -1.571
        # self.joints["RSF"].angMax = 1.176
        #
        # self.joints["RSL"].angMin = -1.123
        # self.joints["RSL"].angMax = 1.571
        # self.joints["RSL"].mapType = 1
        #
        # self.joints["RSH"].angMin = 0
        # self.joints["RSH"].angMax = 1.571
        # self.joints["RSH"].mapType = 2
        #
        # self.joints["REB"].angMin = -1.571
        # self.joints["REB"].angMax = 0
        # self.joints["REB"].mapType = 1

    # Set all servos to default position
    def allToDef(self):
        for k, j in self.joints.items():
            j.setState("def")
        rospy.sleep(1.0)

    # Update joints according to incoming joint target joint states
    def updateJoints(self):
        # This list contains all active joints. If a joint name is removed, it will not get updated. Note that LSH/RSH are currently not contained
        working_joints = ["NF", "NH", "LSF", "LSL", "LEB", "RSF", "RSL", "REB"]
        for joint_name, target_value in target_joint_positions.items():
            J = self.joints[joint_name]
            if J.name in working_joints:
                J.set_to_itp_val(target_value)


# Body class done ================================================================================================


# Main ===========================================================================================================

# Callback function which is executed when the joint-state listener receives new data
def callback(JointState):
    # Copy incoming joint state target data into storage variable
    target_joint_positions["NH"] = JointState.position[0]
    target_joint_positions["NF"] = JointState.position[1]
    target_joint_positions["RSF"] = JointState.position[2]
    target_joint_positions["RSL"] = JointState.position[3]
    target_joint_positions["RSH"] = JointState.position[4]
    target_joint_positions["REB"] = JointState.position[5]
    target_joint_positions["RW"] = JointState.position[6]
    target_joint_positions["RH"] = JointState.position[7]
    target_joint_positions["LSF"] = JointState.position[8]
    target_joint_positions["LSL"] = JointState.position[9]
    target_joint_positions["LSH"] = JointState.position[10]
    target_joint_positions["LEB"] = JointState.position[11]
    target_joint_positions["LW"] = JointState.position[12]
    target_joint_positions["LH"] = JointState.position[13]

    # Update joints according to target joint states
    body.updateJoints()


if __name__ == '__main__':
    # Init node
    rospy.init_node("monkey_listener")
    rospy.loginfo("Monkey listener node has started")

    # Create body object which manages joints
    body = Body()

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
