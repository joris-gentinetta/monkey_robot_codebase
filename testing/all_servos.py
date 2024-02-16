from adafruit_servokit import ServoKit
from time import sleep
import argparse

parser = argparse.ArgumentParser(description='Test all servos')
parser.add_argument('--type', type=str, default=None, help='type of action to perform')
args = parser.parse_args()
kit = ServoKit(channels=16, frequency=333)


for s in range(16):
    kit.servo[s].set_pulse_width_range(900, 2200)
    kit.servo[s].angle = 90

if args.type == 'sweep':
    for s in range(16):
        sleep(0.5)
        kit.servo[s].angle = 20
        sleep(0.5)
        kit.servo[s].angle = 90
        sleep(0.5)
        kit.servo[s].angle = 160
        sleep(0.5)
        kit.servo[s].angle = 90


if args.type == 'wear':
    while True:
        for s in range(16):
            sleep(0.5)
            kit.servo[s].angle = 20
        for s in range(16):
            sleep(0.5)
            kit.servo[s].angle = 90
        for s in range(16):
            sleep(0.5)
            kit.servo[s].angle = 160
        for s in range(16):
            sleep(0.5)
            kit.servo[s].angle = 90


