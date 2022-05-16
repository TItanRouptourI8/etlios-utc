#!/usr/bin/python3


import rospy
import numpy
from std_msgs.msg import Int32,String
from geometry_msgs.msg import PointStamped, Point
import math

class _Getch:
    """Gets a single character from standard input.  Does not echo to the screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


class RobotController:
    def __init__(self):
        self.keypresses = [False, False, False, False]
        self.rpmP = rospy.Publisher('/rpm_wanted', Point)
        self.linear = 0
        self.angular = 0
        self.getch = _Getch()
        print("Starting robotController")


    def updateSpeed(self, linear,angular):

        self.linear = linear
        self.angular = angular

        vd = ((0.165*angular + linear)/0.0275)*(60/(2*math.pi))
        vd = numpy.clip(vd,-180,180)
        vg = ((linear - 0.165*angular)/0.0275)*(60/(2*math.pi))
        vg = numpy.clip(vg, -180, 180)
        print("vitesse roue droite :", vd, " vitesse roue gauche :",vg)
        print("vitesse linéaire :", linear, " vitesse angulaire :",angular)
        r = Point()
        r.x = -vg
        r.y = vd
        r.z = 0
        self.rpmP.publish(r)


    def robotLoop(self):
        key = self.getch()
        if key == "z":
            self.updateSpeed(self.linear + 0.1, self.angular)
        elif key == "s":
            self.updateSpeed(self.linear - 0.1, self.angular)
        elif key == " ":
            self.updateSpeed(0,0)
        elif key == "d":
            self.updateSpeed(self.linear, self.angular - 0.1)
        elif key == "q":
            self.updateSpeed(self.linear, self.angular + 0.1)
        elif key == "l":
            return False
        return True


if __name__ == '__main__':
    rospy.init_node("robotController")
    r = RobotController()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        continuer = r.robotLoop()

        if not continuer:
            print("Quitting program")
            break

