#!/usr/bin/env python

import pygame
import rospy
from control.msg import *


# Define some colors.
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')


# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

''' 
    Get the throttle value to the esc based on two triggers values.
    Throttle ranges from -100 to 100.
'''
def get_throttle(joystick, left_trigger_initialized, right_trigger_intialized):
    
    brake_offset = 1.0 if left_trigger_initialized else 0
    acce_offset = 1.0 if right_trigger_initialized else 0

    brake = (joystick.get_axis(3) + brake_offset) * 50.0
    acceleration = (joystick.get_axis(4) + acce_offset) * 50.0
    throttle = acceleration - brake
    return throttle

''' 
    Return whether or not two triggers where initialized.
    This determine how we calculate the throttle value.
'''
def get_triggers_pressed(joystick):
    return joystick.get_button(6) == 1, joystick.get_button(7) == 1

''' Get steering value. Range from -100 to 100'''
def get_steering(joystick):
    return joystick.get_axis(0) * 100


if __name__ == '__main__':

    pygame.init()

    # Set the width and height of the screen (width, height).
    screen = pygame.display.set_mode((500, 700))

    pygame.display.set_caption("My Game")

    # Loop until the user clicks the close button.
    done = False

    # Used to manage how fast the screen updates.
    clock = pygame.time.Clock()

    # Initialize the joysticks.
    pygame.joystick.init()

    # Get ready to print.
    textPrint = TextPrint()

    rospy.init_node('joystick')
    pub = rospy.Publisher('manual_control', command, queue_size=10)
    rate = rospy.Rate(30)

    left_trigger_initialized = False
    right_trigger_initialized = False

    # -------- Main Program Loop -----------
    while not rospy.is_shutdown():
        #
        # EVENT PROCESSING STEP
        #
        # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION
        for event in pygame.event.get(): # User did something.
            if event.type == pygame.QUIT: # If user clicked close.
                done = True # Flag that we are done so we exit this loop.
            elif event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            elif event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

        #
        # DRAWING STEP
        #
        # First, clear the screen to white. Don't put other drawing commands
        # above this, or they will be erased with this command.
        screen.fill(WHITE)
        textPrint.reset()

        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()

        textPrint.tprint(screen, "Number of joysticks: {}".format(joystick_count))
        textPrint.indent()

        # For each joystick:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            textPrint.tprint(screen, "Joystick {}".format(i))
            textPrint.indent()

            # Get the name from the OS for the controller/joystick.
            name = joystick.get_name()
            textPrint.tprint(screen, "Joystick name: {}".format(name))

            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()
            textPrint.tprint(screen, "Number of axes: {}".format(axes))
            textPrint.indent()

            for i in range(axes):
                axis = joystick.get_axis(i)
                textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(i, axis))
            

            
            textPrint.unindent()

            buttons = joystick.get_numbuttons()
            textPrint.tprint(screen, "Number of buttons: {}".format(buttons))
            textPrint.indent()

            for i in range(buttons):
                button = joystick.get_button(i)
                textPrint.tprint(screen,
                                 "Button {:>2} value: {}".format(i, button))

            textPrint.unindent()

            hats = joystick.get_numhats()
            textPrint.tprint(screen, "Number of hats: {}".format(hats))
            textPrint.indent()

            # Hat position. All or nothing for direction, not a float like
            # get_axis(). Position is a tuple of int values (x, y).
            for i in range(hats):
                hat = joystick.get_hat(i)
                textPrint.tprint(screen, "Hat {} value: {}".format(i, str(hat)))
            textPrint.unindent()

            textPrint.unindent()


            # Convert left trigger and right trigger data into throttle command.
            left_trigger_pressed, right_trigger_pressed = get_triggers_pressed(joystick)
            left_trigger_initialized = left_trigger_initialized or left_trigger_pressed
            right_trigger_initialized = right_trigger_initialized or right_trigger_pressed             
            throttle = get_throttle(joystick, left_trigger_initialized, right_trigger_initialized)

            steering = get_steering(joystick)

            textPrint.tprint(screen, "Throttle: {}".format(throttle))
            textPrint.tprint(screen, "Steering: {}".format(steering))



        #
        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
        #

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

        # Limit to 20 frames per second.
        clock.tick(30)

        rate.sleep()

    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()
