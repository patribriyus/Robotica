#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import math
import os
import cv2
import time
from Robot import Robot
from MapLib import Map2D
import numpy as np
from sample_matching import find_template, match_images, drawMatches2


def movimientoConObstaculos(camino, myMap, robot, xObj, yObj):
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


# Si izqODer = true -> izquierda
def orientarRobot(x, y, th, xObj, yObj, thObj, myMap, robot):
    # primero ir rectos hacia adelante y aumentar (o reducir) la x hasta 2200
    if th != thObj:
        robot.girarRadianesOdom(th*-1)
    if x != xObj:
        robot.moverMetrosOdom((x - xObj) * -1) #si está en -0,6 se mueve 0
        # si está en -0.9 dará -0.3, pero tiene que ir hacia adelante, por eso el -1
        #si está en -0.3 dará 0.3, pero tiene que ir hacia atrás

    # segundo ajustar nuestra y a la posición 1800
    if y < yObj:
        robot.girarRadianesOdom(math.pi/2)
        robot.moverMetrosOdom((y - yObj) * -1)
        robot.girarRadianesOdom(-math.pi / 2)
    elif y > yObj:
        robot.girarRadianesOdom(-math.pi / 2)
        robot.moverMetrosOdom((y - yObj) * -1)
        robot.girarRadianesOdom(math.pi / 2)



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
        ap = argparse.ArgumentParser()
        ap.add_argument("-i", "--image", default=None, help="path to the input image")

        # TODO: definir las variables diametro objetivo.
        # robot.moverCesta("SUBIR") # Se sube la cesta
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
            ap.add_argument("-r", "--robot", default="BB8_s.png", help="target template file")

            # trayectoria en S empezamos en el 600 2600

            camino = myMap.findPath(1, 6, 0, 6)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(0, 6, 0, 4)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(0, 4, 2, 4)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(2, 4, 2, 2)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(2, 2, 1, 2)
            movimientoBasico(camino, myMap, robot)

            # ir a la casilla de finPlan con replan

            camino = myMap.findPath(1, 2, 3, 2)
            movimientoConObstaculos(camino, myMap, robot, 3, 2)

            # perseguir blob

            res = robot.trackObject()
            if res:
                robot.catch()

            # colocarnos en una x, y y theta concreta (la x e y concreta es delante del robot de la izquierda)

            x, y, th = robot.readOdometry()
            orientarRobot(x, y, th, myMap, -0.4, -1.2, 0, robot)

            found = matchingPhoto(ap)
            # si hacemos matching, mirar y salir por la izquierda
            if found:
                #salir por la izquierda
                robot.girarRadianesOdom(math.pi/2)
                robot.moverMetrosOdom(0.4)
                robot.girarRadianesOdom(-math.pi/2)
                robot.moverMetrosOdom(0.8)

            else:
                #movernos a la derecha y salir a la derecha
                orientarRobot(x, y, th, myMap, -0.4, -1.6, 0, robot)
                found = matchingPhoto(ap)
                if not found:
                    #TODO problemas serios
                    # no hemos visto nada, podemos probar a girar sobre nosotros mismos hasta verlo
                    robot.setSpeed(0, 0.2)
                #salir por la derecha

                robot.girarRadianesOdom(-math.pi/2)
                robot.moverMetrosOdom(0.4)
                robot.girarRadianesOdom(math.pi/2)
                robot.moverMetrosOdom(0.8)

        else:
            print("Color negro")
            map_file = "./mapaB_CARRERA.txt"
            myMap = Map2D(map_file)
            myMap.initOjos()
            ap.add_argument("-r", "--robot", default="R2-D2_s.png", help="target template file")

            # trayectoria en S invertida

            camino = myMap.findPath(5, 6, 6, 6)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(6, 6, 6, 4)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(6, 4, 4, 4)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(4, 4, 4, 2)
            movimientoBasico(camino, myMap, robot)

            camino = myMap.findPath(4, 2, 5, 2)
            movimientoBasico(camino, myMap, robot)

            # ir a la casilla de finPlan con replan

            camino = myMap.findPath(5, 2, 3, 2)
            movimientoConObstaculos(camino, myMap, robot, 3, 2)

            # perseguir blob

            res = robot.trackObject()
            if res:
                robot.catch()

            # colocarnos en una x, y y theta concreta y buscar robots

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


def matchingPhoto(ap):
    args = ap.parse_args()
    # default values
    im = None
    mirror = True
    if args.image is not None:
        if not os.path.isfile(args.image):
            print("test image %s does not exist" % args.image);
            return
        im = cv2.imread(args.image, cv2.IMREAD_COLOR)
        mirror = False
    if not os.path.isfile(args.robot):
        print("target template image %s does not exist" % args.robot);
        return
    else:
        target_robot_file = args.robot
    found = find_template(mirror=mirror, img=im, refFilename=target_robot_file)
    return found


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--color", help="color of the ball to track",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)
