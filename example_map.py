# !/usr/bin/python
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

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        myMap.initOjos()
        # myMap.verbose = True
        myMap.drawMap(saveSnapshot=False)

        # you can set verbose to False to stop displaying plots interactively
        # (and maybe just save the snapshots of the map)
        # myMap.verbose = False

        # sample commands to see how to draw the map
        sampleRobotLocations = [[0, 0, 0], [600, 600, 3.14]]
        # this will save a .png with the current map visualization,
        # all robot positions, last one in green
        # myMap.verbose = True
        myMap.drawMapWithRobotLocations(sampleRobotLocations, saveSnapshot=False)

        # this shows the current, and empty, map and an additionally closed connection
        myMap.deleteConnection(0, 0, 0)
        # myMap.verbose = True
        myMap.drawMap(saveSnapshot=False)

        # this will open a window with the results, but does not work well remotely
        # myMap.verbose = True
        sampleRobotLocations = [[200, 200, 3.14 / 2.0], [200, 600, 3.14 / 4.0], [200, 1000, -3.14 / 2.0], ]
        myMap.drawMapWithRobotLocations(sampleRobotLocations, saveSnapshot=False)

        matplotlib.pyplot.close('all')

        # 2. launch updateOdometry thread()
        robot.startOdometry()
        xObj = 0  # TODO pedir por parametro
        yObj = 1  # TODO pedir por parametro

        camino = myMap.findPath(0, 0, xObj, yObj)

        # 3. perform trajectory
        print("datos camino", camino, len(camino))
        while len(camino) != 0:
            i = camino[0]
            camino = np.delete(camino, 0, 0)
            print("datos post-borrado de paso", camino, len(camino))
            
            giro = myMap.go(i[0], i[1])
            print("tengo que girar: ", giro)
            
            # TODO: asegurarse que gira lo que tiene que girar
            robot.girarRadianes(giro)

            hayObstaculo = myMap.detectObstacle()
            print("hayObstaculo?", hayObstaculo)
            if (hayObstaculo):
                camino = myMap.replanPath(xObj, yObj)

            else:
                # TODO: asegurarse que avanza 0.4 metros
                robot.moverMetros(0.4)

                myMap.cambiarPosIni(i[0], i[1])

        # 4. wrap up and close stuff ...
        myMap.disableSensors()
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        myMap.disableSensors()
        robot.stopOdometry()
        print('do something to stop Robot when we Ctrl+C ...')


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="mapa1.txt")
    args = parser.parse_args()
    main(args)
