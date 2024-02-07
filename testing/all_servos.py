from adafruit_servokit import ServoKit
kit = ServoKit(channels=16, frequency=333)
from sleep import sleep

for s in range (16):
    print(s)
    sleep(0.5)
    kit.servo[s].set_pulse_width_range(900,2200)
    kit.servo[s].angle = 0
    sleep(0.2)
    kit.servo[s].angle = 180
    sleep(0.2)
    kit.servo[s].angle = 90
    sleep(0.5)

