#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # ''

# import brickpi3 # import the BrickPi3 drivers
import time  # import the time library for the sleep function
from math import degrees

import brickpi3  # import the BrickPi3 drivers
import sys
import numpy as np
import math
import picamera
from picamera.array import PiRGBArray
from detector import detectorInit
import cv2

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

Gradio = 0.027  # Radio ruedas motoras (m - ahora mismo)
GL = 0.137  # Distancia entre ruedas motoras (m - ahora mismo entre centros ruedas)

ESC = 27

# Datos de la posicion objetiva del blob
x_min = 175.0
x_max = 181.0

y_min = 200.0
y_max = 350.0


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        # Robot construction parameters

        # Crea el fichero txt LOG
        self.LOG = open('LOG.txt', 'w')

        # Velocidad angular de las ruedas Right y Left
        # readianes/seg
        self.WR = 0
        self.WL = 0

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # reset encoder B and C
        self.BP.offset_motor_encoder(self.BP.PORT_B,
                                     self.BP.get_motor_encoder(self.BP.PORT_B))  # reset encoder B (left)
        self.BP.offset_motor_encoder(self.BP.PORT_C,
                                     self.BP.get_motor_encoder(self.BP.PORT_C))  # reset encoder C (right)
        self.BP.offset_motor_encoder(self.BP.PORT_D,
                                     self.BP.get_motor_encoder(self.BP.PORT_D))  # reset encoder D (cesta)

        ##################################################
        # odometry shared memory values (Localizacion)
        self.x = Value('d', 0.0)
        self.y = Value('d', 0.0)
        self.th = Value('d', 0.0)
        self.finished = Value('b', 1)  # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.2  # tiempo entre cada comprobacion de la odometria (thread)

        # set camera ON
        self.cam = picamera.PiCamera()
        self.cam.resolution = (320, 240)
        self.cam.rotation = 180
        self.cam.framerate = 32
        self.rawCapture = PiRGBArray(self.cam, size=(320, 240))

        self.cestaArriba = False

    def setSpeed(self, v, w):
        """ Establece la velocidad lineal y angular correspondiente a las ruedas """

        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...

        matrixVW = np.array([[v], [w]])
        matrixRL = np.array([[1 / Gradio, GL / (2 * Gradio)], [1 / Gradio, -GL / (2 * Gradio)]])
        matrixW = np.dot(matrixRL, matrixVW)

        # Establecer velocidad a ambos motores a la vez

        # Set the motor target speed in degrees per second
        speedDPS_right = degrees(matrixW[0][0])
        speedDPS_left = degrees(matrixW[1][0])

        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)

    def setSpeedCesta(self, v, w):
        """ Establece velocidad al motor de la cesta """

        # compute the speed that should be set in each motor ...

        matrixVW = np.array([[v], [w]])
        matrixRL = np.array([[1 / Gradio, GL / (2 * Gradio)], [1 / Gradio, -GL / (2 * Gradio)]])
        matrixW = np.dot(matrixRL, matrixVW)

        # Set the motor target speed in degrees per second
        speedDPS = degrees(matrixW[1][0])

        self.BP.set_motor_dps(self.BP.PORT_D, speedDPS)

    def readSpeed(self):
        """ Lee las velocidades de las ruedas"""
        matrixRL = np.array([[Gradio / 2, Gradio / 2], [Gradio / GL, -Gradio / GL]])
        [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                    math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]
        x = wl - self.WL
        y = wr - self.WR
        self.WL = wl
        self.WR = wr
        matrixW = np.array([[y / self.P], [x / self.P]])
        matrixVW = np.dot(matrixRL, matrixW)
        
        return matrixVW[0, 0], matrixVW[1, 0]

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())  # additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    def updateOdometry(self):
        """ Actualiza los valores de la odometria por los actuales """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            # TODO: descomentar
            # sys.stdout.write("Update of odometry ...., X=  %.2f, \
            # Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))
            # print("Update of odometry ...., X=  %.2f" %(self.x.value) )

            try:

                difS = 0
                difTH = 0

                self.lock_odometry.acquire()
                v, w = self.readSpeed()

                self.lock_odometry.release()

                if (w == 0):  # Cambiar por w pequeo?
                    difS = v * self.P
                else:
                    difTH = w * self.P
                    difS = (v / w) * difTH

                angulo = self.th.value + (difTH / 2)

                thNueva = self.th.value + difTH
                # si > pi -- th - 2*pi --> giro izda
                # si < -pi -- th +2*pi --> giro dcha
                thNueva = thNueva % (2 * math.pi)
                if thNueva > math.pi:
                    thNueva = thNueva - 2 * math.pi
                if thNueva < -math.pi:
                    thNueva = thNueva + 2 * math.pi
                    # self.th.value=norm_pi(self.th.value)

                self.lock_odometry.acquire()
                self.x.value = self.x.value + difS * math.cos(angulo)
                self.y.value = self.y.value + difS * math.sin(angulo)
                self.th.value = thNueva
                self.lock_odometry.release()

                # Mete los datos en el LOG
                self.LOG.write(str(self.x.value) + ' ' + str(self.y.value)
                               + ' ' + str(self.th.value) + '\n')

            except IOError as error:
                # print(error)
                sys.stdout.write(error)

            # sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))

            # save LOG

            tEnd = time.clock()
            time.sleep(self.P - (tEnd - tIni))

            # print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))
        self.setSpeed(0, 0)
        self.LOG.close()  # Se cierra fichero LOG

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.setSpeed(0, 0)
        cv2.destroyAllWindows()

    def velAng(self, xBlob):
        """Decide si la velocidad angular tiene que ser positiva o negativa"""

        if xBlob < x_min:
            w = 0.2
        elif xBlob > x_max:
            w = -0.2

        self.setSpeed(0.07, w)

    def velLin(self, dBlob):
        """Decide cuando la velocidad lineal tiene que ser ser mas rapida o mas lenta"""

        self.setSpeed(0.07, 0)

    def posObjetiva(self, xBlob, yBlob, dBlob):

        x = xBlob >= x_min and xBlob < x_max
        y = yBlob >= y_min and yBlob < y_max

        return x and y

    def trackObject(self):
        """ Esta funcion persigue la pelota roja hasta una posicion objetivo """

        # Elegimos el umbral de rojo en HSV
        redMin1 = (175, 100, 75)
        redMax1 = (179, 255, 255)
        # Elegimos el segundo umbral de rojo en HSV
        redMin2 = (0, 100, 100)
        redMax2 = (8, 255, 255)

        detector = detectorInit()

        for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            img = img.array
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            k = cv2.waitKey(1) & 0xff
            if k == ESC:  # TODO: cambiar para finalizar bien con ctrl+C
                self.cam.close()
                break

            img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # hacemos la mask y filtramos en la original
            mask1 = cv2.inRange(img_HSV, redMin1, redMax1)
            mask2 = cv2.inRange(img_HSV, redMin2, redMax2)
            mask_red = mask1 + mask2

            # detector finds "dark" blobs by default, so invert image for results with same detector
            keypoints_red = detector.detect(mask_red)

            # documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
            # "x":the x coordinate of each blob in the image.
            # "y":the y coordinate of each blob in the image.
            # "size":the diameter of the circle containing the blob.
            blobVacio = True
            max = 0
            for kp in keypoints_red:
                if (max < kp.size):
                    blobVacio = False
                    max = kp.size
                    print(kp.pt[0], kp.pt[1], kp.size)
                    kp_obj = kp


            im_with_keypoints = cv2.drawKeypoints(img, keypoints_red, np.array([]),
                                                  (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Show mask and blobs found
            cv2.imshow("Keypoints on RED", im_with_keypoints)

            if not blobVacio:  # si ha detectado un blob, entra

                if not self.posObjetiva(kp_obj.pt[0], kp_obj.pt[1], kp_obj.size):
                    print("NO es posicion objetiva")

                    if kp_obj.pt[0] < x_min or kp_obj.pt[0] >= x_max:
                        self.velAng(kp_obj.pt[0])
                    else:
                        self.velLin(kp_obj.size)

                else:
                    print(" Posicion objetiva")
                    break

            else:
                # Si no encuentra la pelota, da vueltas sobre si mismo
                self.setSpeed(0, 0.8)

        return True

    def resetOdom(self):
        """ Esta funcion busca (pero no persigue) la pelota roja hasta una posicion objetivo """

        # Elegimos el umbral de rojo en HSV
        redMin1 = (175, 100, 75)
        redMax1 = (179, 255, 255)
        # Elegimos el segundo umbral de rojo en HSV
        redMin2 = (0, 100, 100)
        redMax2 = (8, 255, 255)

        detector = detectorInit()

        for img in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            img = img.array
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            k = cv2.waitKey(1) & 0xff
            if k == ESC:  # TODO: cambiar para finalizar bien con ctrl+C
                self.cam.close()
                break

            img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # hacemos la mask y filtramos en la original
            mask1 = cv2.inRange(img_HSV, redMin1, redMax1)
            mask2 = cv2.inRange(img_HSV, redMin2, redMax2)
            mask_red = mask1 + mask2

            # detector finds "dark" blobs by default, so invert image for results with same detector
            keypoints_red = detector.detect(mask_red)

            # documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
            # "x":the x coordinate of each blob in the image.
            # "y":the y coordinate of each blob in the image.
            # "size":the diameter of the circle containing the blob.
            blobVacio = True
            max = 0
            for kp in keypoints_red:
                if (max < kp.size):
                    blobVacio = False
                    max = kp.size
                    print(kp.pt[0], kp.pt[1], kp.size)
                    kp_obj = kp

            im_with_keypoints = cv2.drawKeypoints(img, keypoints_red, np.array([]),
                                                  (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Show mask and blobs found
            cv2.imshow("Keypoints on RED", im_with_keypoints)

            if not blobVacio:  # si ha detectado un blob, entra

                if not (170 < kp_obj.pt[0] < 180):
                    print("NO es posicion objetiva")
                    self.setSpeed(0, 0.3)

                else:
                    print(" Posicion objetiva")
                    self.setSpeed(0, 0)
                    break

            else:
                # Si no encuentra la pelota, da vueltas sobre si mismo
                self.setSpeed(0, 0.7)

            self.lock_odometry.acquire()
            self.y.value = 0.0
            self.th.value = 0
            self.lock_odometry.release()

        return True
    
    def setXValue(self,newX):
        self.lock_odometry.acquire()
        self.x.value = newX
        self.lock_odometry.release()
        return None

    def moverCesta(self, movimiento):
        """Mueve la cesta hacia arriba o hacia abajo"""
        if movimiento == "SUBIR":
            self.setSpeedCesta(-0.05, 0)
            time.sleep(0.6)
            self.setSpeedCesta(0.0, 0)
            self.cestaArriba = True

        else:  # "BAJAR"
            self.setSpeedCesta(0.05, 0)
            time.sleep(0.50)
            self.setSpeedCesta(0.0, 0)
            self.cestaArriba = False

    def catch(self):
        """ Tras llegar a la posicion deseada, coge la pelota """
        # Se considera que empieza con ella arriba

        # Bajar cesta
        if self.cestaArriba:
            self.moverCesta("BAJAR")

        else:
            self.moverCesta("SUBIR")
            self.moverCesta("BAJAR")

    def girarRadianesOdom(self, radianes):
        x, y, th = self.readOdometry()

        radObj = th + radianes

        radObj = radObj % (2 * math.pi)
        if radObj > math.pi:
            radObj = radObj - 2 * math.pi
        if radObj < -math.pi:
            radObj = radObj + 2 * math.pi

        if (radianes < 0):
            # derecha
            self.setSpeed(0, -0.75)
        elif (radianes > 0):
            # izquierda
            self.setSpeed(0, 0.75)

        while not (radObj - 0.08 < th < radObj + 0.08):
            time.sleep(self.P)
            x, y, th = self.readOdometry()

        self.setSpeed(0, 0)

    def girarRadianes(self, radianes):
        """ Gira al robot +-X radianes de su posicion """

        metros = (radianes / (2 * math.pi)) * 2 * math.pi * GL / 2
        radianes = (2 * math.pi * metros) / (2 * math.pi * Gradio)

        print("Metros: ", metros)

        # Se calculan los radianes mas actuales de cada una de las ruedas
        [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                    math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]

        if (radianes < 0):  # negativo -> giro a la derecha
            self.setSpeed(0, -0.7)
            radObj = abs(wl) + abs(radianes)

            print("radObj :", radObj)

            while (radObj > abs(wl)):
                time.sleep(self.P)
                [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                            math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]
                print("wl :", wl)

        elif (radianes > 0):  # positivo -> giro a la izquierda
            self.setSpeed(0, 0.7)
            radObj = abs(wr) + radianes

            print("radObj :", radObj)

            while (radObj > abs(wr)):
                time.sleep(self.P)
                [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                            math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]
                print("wr :", wr)

        self.setSpeed(0, 0)

    def moverMetros(self, m):
        """ Mueve al robot +-X metros de su posicion """
        # Segun los metros m introducidos hay que calcular a cuantos radianes
        # del giro de cada una de las ruedas seria
        radianes = (2 * math.pi * m) / (2 * math.pi * Gradio)

        # Se calculan los radianes mas actuales de cada una de las ruedas
        # Solo nos es necesario uno, puesto que se movera 'recto'
        [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                    math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]

        if (radianes < 0):  # negativo -> hacia atras
            self.setSpeed(-0.07, 0)
            radObj = wr + abs(radianes)
            while (radObj > wr):
                time.sleep(self.P)
                [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                            math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]

        elif (radianes > 0):  # positivo -> hacia delante
            self.setSpeed(0.07, 0)
            radObj = wl + radianes
            while (radObj > wl):
                time.sleep(self.P)
                [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                            math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]

        self.setSpeed(0, 0)

    def moverMetrosOdom(self, m):
        """ Mueve al robot +-X metros de su posicion """
        xIni, yIni, thIni = self.readOdometry()
        error = 0.01  # error admitido en el movimiento (en metros)
        v = 0

        if (m < 0):  # negativo -> hacia atras
            v = -0.09
        elif (m > 0):  # positivo -> hacia delante
            v = 0.09

        m = abs(m)  # trabajar con distancias absolutas
        self.setSpeed(v, 0)
        x, y, th = self.readOdometry()
        desp = np.sqrt((x - xIni) ** 2 + (y - yIni) ** 2)  # por trigonometria (hipotenusa),
        # desp es la distancia recorrida

        while not (m - error < desp < m + error):
            time.sleep(self.P)
            x, y, th = self.readOdometry()
            desp = np.sqrt((x - xIni) ** 2 + (y - yIni) ** 2)

        self.setSpeed(0, 0)

    def initLuz(self):
        self.BP.set_sensor_type(self.BP.PORT_4, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

    def sobreQueColorEstamos(self):
        """devuelve True si es blanco False si es negro"""
        return (self.BP.get_sensor(self.BP.PORT_4) < 1800)

    def disableLuz(self):
        """ Unconfigure the sensors, disable the motors,
        and restore the LED to the control of the BrickPi3 firmware."""
        self.BP.reset_all()
