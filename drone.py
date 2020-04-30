from djitellopy import Tello
from controls import run
from hud import get_hud
from pid import PID
import argparse
import cv2.cv2 as cv2
import datetime
import numpy as np
import os
import time
import constants


face_cascade = cv2.CascadeClassifier('cascades/data/haarcascade_frontalface_alt_tree.xml')
recognizer = cv2.face.LBPHFaceRecognizer_create()
ddir = ""

class FrontEnd(object):

    def __init__(self):
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        self.h_pid = PID(0.1, 0.00001, 0.01)
        self.v_pid = PID(0.5, 0.00001, 0.01)
        self.dist_pid = PID(0.1, 0.00001, 0.01)

        self.send_rc_control = False
    def run(self, args):
        run(self, args, lerp, ddir, face_cascade)


    def battery(self):
        return self.tello.get_battery()[:2]

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)

def lerp(a, b, c):
    return a + c * (b - a)

def arguments():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter, add_help=False)
    parser.add_argument('-h', '--help', action='help', default=argparse.SUPPRESS,
                        help='** = required')
    parser.add_argument('-d', '--distance', type=int, default=3,
                        help='use -d to change the distance of the drone. Range 0-6')
    parser.add_argument('-sx', '--saftey_x', type=int, default=100,
                        help='use -sx to change the saftey bound on the x axis . Range 0-480')
    parser.add_argument('-sy', '--saftey_y', type=int, default=55,
                        help='use -sy to change the saftey bound on the y axis . Range 0-360')
    parser.add_argument('-os', '--override_speed', type=int, default=1,
                        help='use -os to change override speed. Range 0-3')
    parser.add_argument('-ss', "--save_session", action='store_true',
                        help='add the -ss flag to save your session as an image sequence in the Sessions folder')
    parser.add_argument('-D', "--debug", action='store_true',
                        help='add the -D flag to enable debug mode. Everything works the same, but no commands will be sent to the drone')

    return parser.parse_args()

def main():
    user_arguments = arguments()
    frontend = FrontEnd()

    # run frontend
    frontend.run(user_arguments)


if __name__ == '__main__':
    main()
