#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
# import numpy as np
import time
import math
from Robot import Robot


# from readLOG import readLOG


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" % (robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory
        # -------------------------------------------------



        # TRAYECTORIA 1
        '''
        print("Start : %s" % time.ctime())

        # Gira sobre si mismo
        robot.setSpeed(0,-0.9)
        time.sleep(3)

        robot.setSpeed(0.2,1.0)
        time.sleep(3.5)
        robot.setSpeed(0.2,-1.0)
        time.sleep(7)
        robot.setSpeed(0.2,1.0)
        time.sleep(3.5)

        print("End : %s" % time.ctime())
        '''

        v = 0.1
        w = 0.1
        t = 4
        robot.setSpeed(v, 0)
        time.sleep(t)
        x, y, th = robot.readOdometry()

        robot.setSpeed(v, w)
        time.sleep((v*t - x)/v) #S0/v0
        x, y, th = robot.readOdometry()
        print(x)
        '''
        robot.setSpeed(0.2, 0)
        time.sleep(4)
        robot.setSpeed(0,0)
        print(robot.readOdometry())
        '''

        # -------------------------------------------------

        #TRAYECTORIA 2  r:10cm R:20cm L:50cm
        #tg alpha = (R-r)/L
        '''r=0.1
        R=0.2
        L=0.5
        alpha= math.atan((R-r)/L)
        #Girar sobre si mismo 90 izda
        robot.setSpeed(0, 1.5)
        time.sleep(1.1)
        #Cuarto circulo -alpha a dcha
        t=2
        w=math.radians(90-alpha)/t
        v=r*w
        robot.setSpeed(v,-w)
        time.sleep(t)
        # Recto hipotenusa (sqrt((R-r)^2+L^2))
        t=5
        v = math.sqrt(((R-r)*(R-r))+(L*L))/t
        robot.setSpeed(v,0)
        time.sleep(t)
        #Semicirculo + alpha a dcha
        t= 3
        w = math.radians(180 + alpha) / t
        v = R * w
        robot.setSpeed(v, -w)
        time.sleep(t)
        # Recto hipotenusa (sqrt((R-r)^2+L^2))
        t = 5
        v = math.sqrt(((R - r) * (R - r)) + (L * L)) / t
        robot.setSpeed(v, 0)
        time.sleep(t)
        #Cuarto circulo - alpha a dcha
        t=2
        w = math.radians(90-alpha)/2  # 1 segundo
        v = r * w
        robot.setSpeed(v, -w)
        time.sleep(1)
        robot.setSpeed(0, 0)


        print("hola1")

        robot.lock_odometry.acquire()
        print("hola")
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        print("adios")
        robot.lock_odometry.release()
        '''
        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)


