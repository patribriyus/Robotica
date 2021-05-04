#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import cv2
import time
from Robot import Robot
from MapLib import Map2D
import numpy as np
import picamera
from picamera.array import PiRGBArray
from sample_matching import match_images,drawMatches2

DEBUG = 1

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

#Si izqODer = true -> izquierda
def orientarRobot(x, y, th, myMap, robot):
    #situar robot hacia arriba
    robot.girarRadianesOdom(-th)
    #Situar en eje horizontal (y=1.2 B| -1.2 A)
    yObjA=-1.2
    yObjB=1.2
    if(y < yObjB): #mapab- ir hacia la izquierda (thObj=pi/2)
        giro = np.pi/2
        mov = abs(yObjB) - abs(y)
    else: #mapaB - ir hacia la derecha(thObj=-pi/2)
        giro = -np.pi/2
        mov = abs(y) - abs(yObjB) #TODO: revisar para mapaA

    robot.girarRadianesOdom(giro)
    robot.moverMetrosOdom(mov)
    print('ORIENTADO1')
    x, y, th = robot.readOdometry()
    print("Update of odometry ...., X=  %.2f, Y=  %.2f, th=  %.2f \n" % (x, y, th))

    x,y,th = robot.readOdometry()
    robot.girarRadianesOdom(-th)    # situar el robot hacia arriba

    #situar en eje vertical (x=-0.4)

    if x > -0.4: #retroceder
        robot.moverMetrosOdom(abs(x)-0.4)
    else: #avanzar
        robot.moverMetrosOdom(abs(x)-0.4)
    # si nuestra y es menor, girar a la izquierda, si es mayor, a la derecha
    x, y, th = robot.readOdometry()
    print('ORIENTADO2')
    print("Update of odometry ...., X=  %.2f, Y=  %.2f, th=  %.2f \n" % (x, y, th))
    #por ultimo, mirar de frente e intentaremos matching


    return None

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

def robotDetectado(robot,refFilename):
    # default values
    print("Looking for reference image : ", refFilename)
    imReference = cv2.imread(refFilename, cv2.IMREAD_COLOR)

    print("**** processing PI-CAM image file ****")
    fR=robot.cam.framerate
    robot.cam.framerate = 10  # less frame rate, more light BUT needs to go slowly (or stop)
    rawCapture = PiRGBArray(robot.cam)

    # allow the camera to warmup
    time.sleep(0.2)
    found = False
    #TODO: probar varias veces si no ve imagen?
    #while not found:
    #t1 = time.time() #TODO: dejar tiempos?
    robot.cam.capture(rawCapture, format="bgr")
    frame = rawCapture.array

    #t2 = time.time()
    found = match_images(imReference, frame)
    print("la encontr 0:", found)

    #t3 = time.time()
    #cv2.imshow("INLIERS", frame)
    #cv2.waitKey(0)
   # print("time to match %.2f" % (t3 - t2))
    rawCapture.truncate(0)

    robot.cam.framerate = fR # dejar valor anterior

    return found

def main(args):
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()
        # 1. launch updateOdometry thread()
        robot.startOdometry()
        ap = argparse.ArgumentParser()
        # TODO: definir las variables diametro objetivo.
        robot.moverCesta("SUBIR") # Se sube la cesta
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

            # ir a la casilla de finPlan con replan

            camino = myMap.findPath(1, 2, 3, 2)
            movimientoConObstaculos(camino, myMap, robot, 3, 2)

            #TODO: plot camino

            # perseguir blob

            res = robot.trackObject()
            if res:
                robot.catch()

            # colocarnos en una x, y y theta concreta

            x, y, th = robot.readOdometry()

            orientarRobot(x, y, th, myMap, robot)

            # buscar robots

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
            print("ole ole", found)
            # si hacemos matching, mirar y salir por la izquierda

        else:
            print("Color negro")
            map_file = "mapaB_CARRERA.txt"
            myMap = Map2D(map_file)
            myMap.initOjos()
            im_file="BB8_s.png"

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
            # hacer entrar al robot en la zona de la pelota
            x,y,th = robot.readOdometry()
            robot.girarRadianesOdom(-th) #robot orientado hacia arriba
            robot.moverMetrosOdom(0.4)

            # perseguir blob
            res = robot.trackObject()
            if res:
                robot.catch()
            #TODO: quitar
            '''
            #print('-----------------------pelota cogida--------------------')
            camino = myMap.findPath(5, 6, 4, 6)
            movimientoBasico(camino, myMap, robot)
            camino = myMap.findPath(4, 6, 3, 3)
            movimientoBasico(camino, myMap, robot)
            # colocarnos en una x, y y theta concreta y buscar robots
            x, y, th = robot.readOdometry()
            '''
            #orientar robot
            print("ODOMETRIA ANTES ORIENTAR ...., X=  %.2f, Y=  %.2f, th=  %.2f \n" % (x, y, th))
            orientarRobot(x, y, th, myMap, robot)
            print('-----------------------ROBOT ORIENTADO--------------------')
            # buscar robots

            found = robotDetectado(robot, im_file)
            print("ENCONTRADO", found)

            if found:# si hacemos matching, mirar y salir por la derecha
                robot.girarRadianesOdom(-np.pi/2)
                robot.moverMetrosOdom(0.4)
                robot.girarRadianesOdom(np.pi/2)
                robot.moverMetrosOdom(0.4)
            else:
                robot.girarRadianesOdom(np.pi/2)
                robot.moverMetrosOdom(0.8)
                robot.girarRadianesOdom(-np.pi/2)
                robot.moverMetrosOdom(0.4)



            #TODO: salir
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


