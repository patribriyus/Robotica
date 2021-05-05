#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import cv2
import time

import sample_matching
from Robot import Robot
from MapLib import Map2D
import numpy as np
import picamera
from picamera.array import PiRGBArray
from sample_matching import match_images, drawMatches2

DEBUG = 1


def pegarseALaPared(myMap, robot):
    robot.setSpeed(0.07, 0.0)
    while (myMap.getDistanciaOjos() > 20):
        continue


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
        # myMap.cambiarPosIni(i[0], i[1])
        else:
            robot.moverMetrosOdom(0.4)

            myMap.cambiarPosIni(i[0], i[1])


# Si izqODer = true -> izquierda
def orientarRobot(x, y, th, myMap, robot):
    # situar robot hacia arriba
    robot.girarRadianesOdom(-th)
    # Situar en eje horizontal (y=1.2 B| -1.2 A)
    yObjA = -1.2
    # yObjB = 1.2 #TODO:parametrizar
    yObjB = yObjA
    if (y < yObjB):  # mapab- ir hacia la izquierda (thObj=pi/2)
        giro = np.pi / 2
        # mov = abs(yObjB) - abs(y) #MapaB
        mov = abs(y) - abs(yObjA)  # MapaB
    else:  # mapaB - ir hacia la derecha(thObj=-pi/2)
        giro = -np.pi / 2
        # mov = abs(y) - abs(yObjB)  # mapaB
        mov = abs(yObjA) - abs(y)  # mapaA

    robot.girarRadianesOdom(giro)
    robot.moverMetrosOdom(mov)
    print('ORIENTADO1')
    x, y, th = robot.readOdometry()
    print("Update of odometry ...., X=  %.2f, Y=  %.2f, th=  %.2f \n" % (x, y, th))

    x, y, th = robot.readOdometry()
    robot.girarRadianesOdom(-th)  # situar el robot hacia arriba

    # situar en eje vertical (x=0)
    robot.moverMetrosOdom(abs(x) - 0.1)

    x, y, th = robot.readOdometry()
    print('ORIENTADO2')
    print("Update of odometry ...., X=  %.2f, Y=  %.2f, th=  %.2f \n" % (x, y, th))
    # por ultimo, mirar de frente e intentaremos matching

    return None


def movimientoBasico(camino, myMap, robot):
    while len(camino) != 0:
        i = camino[0]
        camino = np.delete(camino, 0, 0)
        print("datos post-borrado de paso", camino, len(camino))

        giro = myMap.go(i[0], i[1])
        print("tengo que girar: ", giro)

        robot.girarRadianesOdom(giro)

        robot.moverMetrosOdom(0.38)

        myMap.cambiarPosIni(i[0], i[1])


def robotDetectado(robot, refFilename):
    # default values
    print("Looking for reference image : ", refFilename)
    imReference = cv2.imread(refFilename, cv2.IMREAD_COLOR)

    print("**** processing PI-CAM image file ****")
    fR = robot.cam.framerate
    robot.cam.framerate = 10  # less frame rate, more light BUT needs to go slowly (or stop)
    rawCapture = PiRGBArray(robot.cam)

    # allow the camera to warmup
    time.sleep(0.2)
    found = False
    # TODO: probar varias veces si no ve imagen?
    # while not found:
    # t1 = time.time() #TODO: dejar tiempos?
    robot.cam.capture(rawCapture, format="bgr")
    frame = rawCapture.array

    # t2 = time.time()
    found = match_images(imReference, frame)
    print("la encontr 0:", found)

    # t3 = time.time()
    cv2.imshow("INLIERS", frame)
    # cv2.waitKey(0)
    # print("time to match %.2f" % (t3 - t2))
    rawCapture.truncate(0)

    robot.cam.framerate = fR  # dejar valor anterior

    return found


def main(args):
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()
        # 1. launch updateOdometry thread()
        robot.startOdometry()
        ap = argparse.ArgumentParser()
        # TODO: definir las variables diametro objetivo.
        robot.moverCesta("SUBIR")  # Se sube la cesta
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
            im_file = "R2-D2_s.png"
            ap.add_argument("-r", "--robot", default="R2D2_s.png", help="target template file")

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

            cinta = robot.sobreQueColorEstamos()
            while (not cinta):
                robot.moverMetrosOdom(-0.02)
                cinta = robot.sobreQueColorEstamos()

            robot.moverMetrosOdom(0.10)

            robot.resetOdom()

            x, y, th = robot.readOdometry()
            print("ODojmetria desppues de reset", x, y, th)
            robot.girarRadianesOdom(np.pi / 2)

            # ir a la casilla de finPlan con replan

            camino = myMap.findPath(1, 2, 3, 2)
            movimientoConObstaculos(camino, myMap, robot, 3, 2)
            # hacer entrar al robot en la zona de la pelota
            x, y, th = robot.readOdometry()
            robot.girarRadianesOdom(-th)  # robot orientado hacia arriba
            robot.moverMetrosOdom(0.4)
            robot.girarRadianesOdom(-np.pi / 4)  # robot diagonal
            robot.moverMetrosOdom(0.4)

            # perseguir blob
            res = robot.trackObject()
            if res:
                robot.catch()

            x, y, th = robot.readOdometry()
            # orientar robot
            print("ODOMETRIA ANTES ORIENTAR ...., X=  %.2f, Y=  %.2f, th=  %.2f \n" % (x, y, th))
            orientarRobot(x, y, th, myMap, robot)
            print('-----------------------ROBOT ORIENTADO--------------------')
            # buscar robots
            x, y, th = robot.readOdometry()
            robot.girarRadianesOdom(-th)
            found = False
            '''while th > -np.pi/4 and not found:
                found = robotDetectado(robot, im_file)
                robot.girarRadianesOdom(-np.pi/16)
                x, y, th = robot.readOdometry()
            '''
            found = robotDetectado(robot, im_file)
            print("ENCONTRADO", found)

            x, y, th = robot.readOdometry()
            robot.girarRadianesOdom(-th)
            if found:  # si hacemos matching, mirar y salir por la izquierda
                robot.girarRadianesOdom(np.pi / 2)
                pegarseALaPared(myMap, robot)
                robot.girarRadianesOdom(-np.pi / 2)
            else:
                robot.girarRadianesOdom(-np.pi / 2)
                pegarseALaPared(myMap, robot)
                robot.girarRadianesOdom(np.pi / 2)

            res = robot.sobreQueColorEstamos()
            robot.setSpeed(0.07, 0)
            while (not res):
                res = robot.sobreQueColorEstamos()

            robot.setSpeed(0, 0)

            robot.moverMetrosOdom(0.2)


        else:
            print("Color negro")
            map_file = "mapaB_CARRERA.txt"
            myMap = Map2D(map_file)
            myMap.initOjos()
            # im_file = "BB8_s.png"
            im_file = "R2-D2_s.png"

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

            cinta = robot.sobreQueColorEstamos()
            while (not cinta):
                robot.moverMetrosOdom(-0.02)
                cinta = robot.sobreQueColorEstamos()

            robot.moverMetrosOdom(0.10)

            robot.resetOdom()

            x, y, th = robot.readOdometry()
            print("ODojmetria desppues de reset", x, y, th)
            robot.girarRadianesOdom(-np.pi / 2)

            # ir a la casilla de finPlan con replan

            camino = myMap.findPath(5, 2, 3, 2)
            movimientoConObstaculos(camino, myMap, robot, 3, 2)
            # hacer entrar al robot en la zona de la pelota
            x, y, th = robot.readOdometry()
            robot.girarRadianesOdom(-th)  # robot orientado hacia arriba
            robot.moverMetrosOdom(0.4)
            robot.girarRadianesOdom(np.pi / 4)  # robot diagonal
            robot.moverMetrosOdom(0.4)

            # perseguir blob
            res = robot.trackObject()
            if res:
                robot.catch()


            x, y, th = robot.readOdometry()
            # orientar robot
            print("ODOMETRIA ANTES ORIENTAR ...., X=  %.2f, Y=  %.2f, th=  %.2f \n" % (x, y, th))
            orientarRobot(x, y, th, myMap, robot)
            print('-----------------------ROBOT ORIENTADO--------------------')
            # buscar robots
            x, y, th = robot.readOdometry()
            robot.girarRadianesOdom(-th)
            found = False
            '''while th > -np.pi/4 and not found:
                found = robotDetectado(robot, im_file)
                robot.girarRadianesOdom(-np.pi/16)
                x, y, th = robot.readOdometry()
            '''
            found = robotDetectado(robot, im_file)
            print("ENCONTRADO", found)

            x, y, th = robot.readOdometry()
            robot.girarRadianesOdom(-th)
            if found:  # si hacemos matching, mirar y salir por la derecha
                robot.girarRadianesOdom(-np.pi / 2)
                pegarseALaPared(myMap, robot)
                robot.girarRadianesOdom(np.pi / 2)
            else:
                robot.girarRadianesOdom(np.pi / 2)
                pegarseALaPared(myMap, robot)
                robot.girarRadianesOdom(-np.pi / 2)

            res = robot.sobreQueColorEstamos()
            robot.setSpeed(0.07, 0)
            while (not res):
                res = robot.sobreQueColorEstamos()

            robot.setSpeed(0, 0)

            robot.moverMetrosOdom(0.2)
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


