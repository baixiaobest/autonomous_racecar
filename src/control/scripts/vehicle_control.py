#!/usr/bin/env python

import rospy
from control.msg import *
import RPi.GPIO as GPIO

SERVO_PIN = 18
SERVO_FREQUENCY = 100
# 1ms min servo pulse
SERVO_MIN = 10.0
# 2ms max servo pulse
SERVO_MAX = 20.0
SERVO_RANGE = 10.0
# 1.5ms neutral servo position
SERVO_NEUTRAL = 15.0

# Steering range of ros message.
STEERING_RANGE = 200.0

# Object that controls the servo pwm.
servo = None

def setup_GPIO():
    global servo

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    # Servo pin is running 100Hz.
    servo = GPIO.PWM(SERVO_PIN, 100)
    # 15% duty cycle, 1.5ms pulses at 100Hz.
    servo.start(SERVO_NEUTRAL)

def map_steering_to_servo(steering):
    return (steering / STEERING_RANGE * SERVO_RANGE) + SERVO_MIN


def command_received(cmd):
    global servo
    servo.ChangeDutyCycle(map_steering_to_servo(cmd.steering))

if __name__ == '__main__':
    setup_GPIO()

    rospy.init_node('vehicle_control_node')
    
    rospy.Subscriber('vehicle_control', command, command_received)

    rospy.spin()
    
