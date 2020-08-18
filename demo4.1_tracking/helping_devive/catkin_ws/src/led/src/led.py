#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time

BLUE_0 = 6
BLUE_1 = 13
YELLOW_0 = 19
YELLOW_1 = 26
RED_0 = 12
RED_1 = 16
GREEN_0 = 20
GREEN_1 = 21

LEDs = [BLUE_0, BLUE_1, YELLOW_0, YELLOW_1, RED_0, RED_1, GREEN_0, GREEN_1]

GPIO.setmode(GPIO.BCM)
for i in range(8):
    GPIO.setup(LEDs[i], GPIO.OUT)


def off():
    for i in range(8):
        GPIO.output(LEDs[i], GPIO.LOW)


rospy.init_node("led", anonymous=False)
rate = rospy.Rate(2)
count = 0
while not rospy.is_shutdown():
    off()
    GPIO.output(LEDs[count], GPIO.HIGH)
    count += 1
    if count == 8:
        count = 0
    rate.sleep()
