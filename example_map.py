#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
from Robot import Robot
import time

import matplotlib
matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from MapLib import Map2D

# NOTES ABOUT TASKS to DO in P4:
# 1)findPath(x1,y1, x2,y2),   fillCostMatrix(), replanPath () --> should be methods from the new Map2D class
# 2) go(x,y) and detectObstacle() could be part of your Robot class (depending how you have implemented things)
# 3) you can change these method signatures if you need, depending how you have implemented things


def main(args):
    """
    Example to load "mapa1.txt"
    """

    try:
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile;
        # Instantiate Odometry with your own files from P2/P3
        robot = Robot()
        # ...

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        #myMap.verbose = True
        myMap.drawMap(saveSnapshot=False)

        # you can set verbose to False to stop displaying plots interactively
        # (and maybe just save the snapshots of the map)
        # myMap.verbose = False

        # sample commands to see how to draw the map
        sampleRobotLocations = [ [0,0,0], [600, 600, 3.14] ]
        # this will save a .png with the current map visualization,
        # all robot positions, last one in green
        #myMap.verbose = True
        myMap.drawMapWithRobotLocations( sampleRobotLocations, saveSnapshot=False )

        # this shows the current, and empty, map and an additionally closed connection
        myMap.deleteConnection(0,0,0)
        #myMap.verbose = True
        myMap.drawMap(saveSnapshot=False)

        # this will open a window with the results, but does not work well remotely
        #myMap.verbose = True
        sampleRobotLocations = [ [200, 200, 3.14/2.0], [200, 600, 3.14/4.0], [200, 1000, -3.14/2.0],  ]
        myMap.drawMapWithRobotLocations( sampleRobotLocations, saveSnapshot=False )

        matplotlib.pyplot.close('all')
        
        # 2. launch updateOdometry thread()
        robot.startOdometry()
        camino = myMap.findPath(x_ini, y_ini, x_end, y_end) # TODO

        # 3. perform trajectory
        # robot.setSpeed(1,1) ...     
        for i in camino:
            
            giro = myMap.go(i[0], i[1])
            
            # TODO: asegurarse que gira 'giro' radianes
            robot.setSpeed(0, 0.3)
            
            hayObstaculo = myMap.detectObstacle()
            if(hayObstaculo):
                camino = myMap.replanPath()
            
            else:
                # TODO: asegurarse que avanza 40cm's
                robot.setSpeed(0.07, 0)
                
            #TODO SET SPEEDS DE VELOCIDAD ANGULAR,
            # COMPROBAR OBJETO DELANTE, DESPUES VELOCIDAD LINEAL
            # Y COMPROBAR MEDIANTE ODOMETRIA SU LLEGADA
            #arriba = x+40, derecha es y-40, abajo x-40, izq y+40

            # check if there are close obstacles
            # deal with them...
            #TODO CADA PASO, SIEMPRE CADA PASO, OSEA QUE TENEMOS QUE DEAL CON EL OBSTACULO QUE NOS PPUEDAN PONER
            # MIENTRAS HACEMOS ESTE PASO
            #Avoid_obstacle(...) OR RePlanPath(...)

        # 4. wrap up and close stuff ...
        # TODO: This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
    #    robot.stopOdometry()
        print('do something to stop Robot when we Ctrl+C ...')


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="mapa1.txt")
    args = parser.parse_args()
    main(args)
