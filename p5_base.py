#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time
from Robot import Robot
from MapLib import Map2D
import numpy as np

def movimientoConObstaculos(camino, myMap, robot):
    while len(camino) != 0:
            i = camino[0]
            camino = np.delete(camino, 0, 0)
            print("datos post-borrado de paso", camino, len(camino))

            giro = myMap.go(i[0], i[1])
            print("tengo que girar: ", giro)

            robot.girarRadianesOdom(giro)

            hayObstaculo = myMap.detectObstacle()
            print("hayObstaculo?", hayObstaculo)
            if (hayObstaculo):
                camino = myMap.replanPath(xObj, yObj)

            else:
                robot.moverMetrosOdom(0.4)

                myMap.cambiarPosIni(i[0], i[1])

def movimientoBasico(camino, myMap, robot):
    while len(camino) != 0:
            i = camino[0]
            camino = np.delete(camino, 0, 0)
            print("datos post-borrado de paso", camino, len(camino))

            giro = myMap.go(i[0], i[1])
            print("tengo que girar: ", giro)

            robot.girarRadianesOdom(giro)

            robot.moverMetrosOdom(0.4)

            myMap.cambiarPosIni(i[0], i[1])

def main(args):
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()
        # 1. launch updateOdometry thread()
        robot.startOdometry()

        #TODO: definir las variables diametro objetivo.
        #robot.moverCesta("SUBIR") # Se sube la cesta
        # 2. Loop running the tracking until ??, then catch the ball
        # TO-DO: ADD to the Robot class a method to track an object, given certain parameters
        # for example the different target properties we want (size, position, color, ..)
        # or a boolean to indicate if we want the robot to catch the object or not
        # At least COLOR, the rest are up to you, but always put a default value.
        robot.initLuz()
        time.sleep(1)
        res = robot.sobreQueColorEstamos()
        if res:
            print("Color blanco")
            map_file = "./mapaA_CARRERA.txt"
            myMap = Map2D(map_file)
            myMap.initOjos()



            #TODO trayectoria en S empezamos en el 600 2600

            camino = myMap.findPath(1, 6, 0, 6)

            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(0, 6, 0, 4)

            movimientoBasico(camino, myMap, robot)
            camino = myMap.findPath(0, 4, 2, 4)


            movimientoBasico(camino, myMap, robot)
            camino = myMap.findPath(2, 4, 2, 2);

            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(2, 2, 1, 2);

            movimientoBasico(camino, myMap, robot)




        else:
            print("Color negro")
            map_file = "./mapaB_CARRERA.txt"
            myMap = Map2D(map_file)
            myMap.initOjos()

            #TODO trayectoria en S invertida

            camino = myMap.findPath(5, 6, 6, 6)
            movimientoBasico(camino, myMap, robot)
            camino = myMap.findPath(6, 6, 6, 4)
            movimientoBasico(camino, myMap, robot)
            camino = myMap.findPath(6, 4, 4, 4)
            movimientoBasico(camino, myMap, robot)
            camino = myMap.findPath(4, 4, 4, 2);
            movimientoBasico(camino, myMap, robot)
            camino = myMap.findPath(4, 2, 5, 2);
            movimientoBasico(camino, myMap, robot)


        robot.disableLuz()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.setSpeed(0, 0)
        robot.disableLuz()
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--color", help="color of the ball to track",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)


