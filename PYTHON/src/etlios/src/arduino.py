#!/usr/bin/python3
import serial
import traceback
import json
import time
import keyboard
import rospy
from std_msgs.msg import Int32,String,Float32MultiArray
from geometry_msgs.msg import PointStamped, Point
from statistics import mean



class Command:
    def __init__(self, key, data, needanswer = False):
        self.key = key
        self.data = data
        self.needanswer = needanswer

    def toString(self):
        return self.key + " " + self.data + "\n"

class Parser:
    def __init__(self):
        self.arduinoReady = False



    def parseArduinoAnswer(self, answer):
        if answer == "STARTED":
            print("arduino ready to go")
            self.arduinoReady = True
        else:
            data = answer.split()
            if(int(data[0]) == 8) : # we have RPM values
                print(data)
                return ['rpms', [float(data[1]), float(data[2])]]

            return answer

        return None


class Communication:
    def __init__(self):
        self.serial = None
        self.waiting = False
        self.usPublisher = rospy.Publisher('ultrasonic',Int32)
        self.rpmPublisher = rospy.Publisher('rpms', PointStamped)
        self.parser = Parser()
        self.lines = []

    def begin(self, port = '/dev/ttyUSB1', baudRate = 115200, timeout = 0.02):
        try:
            self.serial = serial.Serial(port, baudRate,timeout = timeout)
            self.serial.reset_input_buffer()
            return True 
        except  serial.SerialException as e:
            print(e)
            return False
    
    def sendCommand(self, string : str):
        self.serial.write(string.encode())

    def communicationloop(self):
        self.lines = []
        while self.serial.in_waiting > 0:
            self.lines.append(self.serial.readline().decode('utf-8').rstrip())




        if len(self.lines) != 0 :
            for answer in self.lines:
                self.waiting = False


                c = self.parser.parseArduinoAnswer(answer=answer)
                if c is not None :
                    if c[0] == "rpms":
                        print("rpm value")
                        r = Point()
                        r.x= c[1][0]
                        r.y= c[1][1]
                        r.z = 0
                        p = PointStamped()
                        p.point = r
                        p.header.stamp = rospy.Time.now()
                        self.rpmPublisher.publish(p)

        if self.parser.arduinoReady and not self.waiting:

            self.waiting = True
            self.sendCommand("0831\n")




class RobotCommandController:

    def __init__(self, com : Communication):
        self.com = com
        self.commandPublisher = rospy.Publisher('/robotCommand', String)
        self.commandSubscriber = rospy.Subscriber('/robotCommand', String, self.gotCommand)

    def gotCommand(self, command):
        print(command.data)
        self.com.sendCommand(command.data + "\n")

if __name__ == '__main__':

    try:
        c = Communication()
        r = RobotCommandController(c)
        c.begin()
        rospy.init_node("etlios")
        rate = rospy.Rate(20)
        print("coucou")

        while not rospy.is_shutdown():
            c.communicationloop()
            rate.sleep()
    except rospy.ROSInterruptException:
        print("unable to start rospy")
        pass
