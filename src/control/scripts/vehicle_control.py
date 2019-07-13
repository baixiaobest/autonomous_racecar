#!/usr/bin/env python

import rospy
from control.msg import *
import RPi.GPIO as GPIO

SERVO_PIN = 18
SERVO_FREQUENCY = 100
# 1.2ms of pulse range
SERVO_RANGE = 12.0
# 1.5ms neutral servo position
SERVO_NEUTRAL = 13.0

# Steering range of ros message.
STEERING_RANGE = 200.0

THROTTLE_PIN = 13
THROTTLE_FREQUENCY = 100
# 1ms of pulse range
ESC_NEUTRAL = 13.0
ESC_ACCEL_RANGE = 3 # Max 7
ESC_BRAKE_RANGE = 3 # Max 3
ESC_RANGE = ESC_ACCEL_RANGE + ESC_BRAKE_RANGE

# Throttle range of ros message
THROTTLE_RANGE = 100


# Object that controls the servo pwm.
servo = None

# Object that controls the throttle.
throttle = None

def setup_GPIO():
    global servo, throttle

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    # Servo pin is running 100Hz.
    servo = GPIO.PWM(SERVO_PIN, SERVO_FREQUENCY)
    # 15% duty cycle, 1.5ms pulses at 100Hz.
    servo.start(SERVO_NEUTRAL)

    GPIO.setup(THROTTLE_PIN, GPIO.OUT)
    throttle = GPIO.PWM(THROTTLE_PIN, THROTTLE_FREQUENCY)
    throttle.start(ESC_NEUTRAL)

def map_steering_to_servo(steering):
    return (steering / STEERING_RANGE * SERVO_RANGE) + SERVO_NEUTRAL

def map_throttle_to_pwm(throttle):
    if throttle > 0:
        return throttle / THROTTLE_RANGE * ESC_ACCEL_RANGE + ESC_NEUTRAL
    else:
        return throttle / THROTTLE_RANGE * ESC_BRAKE_RANGE + ESC_NEUTRAL


def command_received(cmd):
    global servo, throttle

    servo.ChangeDutyCycle(map_steering_to_servo(cmd.steering))
    throttle.ChangeDutyCycle(map_throttle_to_pwm(cmd.throttle))


if __name__ == '__main__':
    setup_GPIO()

    rospy.init_node('vehicle_control_node')
    
    rospy.Subscriber('vehicle_control', command, command_received)

    rospy.spin()
    
