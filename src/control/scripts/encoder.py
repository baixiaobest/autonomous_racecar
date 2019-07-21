#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

PIN_A = 27
PIN_B = 22

pin_B_state = GPIO.HIGH

class Encoder:
    def __init__(self, tick_to_meter=1.0):
        self.tick_to_meter = float(tick_to_meter)
        self.total_ticks = 0

    def increment(self):
        self.total_ticks += 1

    def decrement(self):
        self.total_ticks -= 1

    def get_ticks(self):
        return self.total_ticks

    def get_distance(self):
        return self.total_ticks * self.tick_to_meter

# Instance of encoder to manage encoder data.
encoder = Encoder()


def setup_GPIO():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_A, GPIO.IN)
    GPIO.setup(PIN_B, GPIO.IN)

    GPIO.add_event_detect(PIN_A, GPIO.FALLING, callback=encoder_callback, bouncetime=10)
    #GPIO.add_event_detect(PIN_B, GPIO.BOTH, callback=pin_B_callback)

def pin_B_callback(pin):
    global pin_B_state
    pin_B_state = GPIO.input(PIN_B)

def encoder_callback(pin):
    global encoder, pin_B_state 

    encoder.increment()

    '''
    if pin_B_state == GPIO.HIGH:
        encoder.increment()
    elif pin_B_state == GPIO.LOW:
        encoder.decrement()
    '''

if __name__=='__main__':
    try:
        setup_GPIO()
        rospy.init_node('encoder')
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print encoder.get_distance()
            rate.sleep()
        
    finally:
        GPIO.cleanup()
