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
from seguimiento_cam import camInit

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

Gradio = 0.027  # Radio ruedas motoras (m - ahora mismo)
GL = 0.137  # Distancia entre ruedas motoras (m - ahora mismo entre centros ruedas)


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        ######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters

        # Crea el fichero txt LOG
        self.LOG = open('LOG.txt', 'w')

        # Velocidad angular de las ruedas Right y Left
        # readianes/seg
        self.WR = 0
        self.WL = 0
        # self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))  # reset encoder B
        self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C))  # reset encoder C

        ##################################################
        # odometry shared memory values (Localizacion)
        self.x = Value('d', 0.0)
        self.y = Value('d', 0.0)
        self.th = Value('d', 0.0)
        self.finished = Value('b', 1)  # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        # ejemplo de uso
        # self.lock_odometry.acquire()
        # print('hello world', i)
        # self.lock_odometry.release()

        # odometry update period --> UPDATE value!
        self.P = 0.2  # tiempo entre cada comprobacion de la odometria (thread)

    def setSpeed(self, v, w):
        """ To be filled - These is all dummy sample code """


        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...

        matrixVW = np.array([[v], [w]])
        matrixRL = np.array([[1 / Gradio, GL / (2 * Gradio)], [1 / Gradio, -GL / (2 * Gradio)]])
        matrixW = np.dot(matrixRL, matrixVW)

        # Establecer velocidad a ambos motores a la vez
        # speedPower = v
        # BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        # Set the motor target speed in degrees per second
        speedDPS_right = degrees(matrixW[0][0])
        speedDPS_left = degrees(matrixW[1][0])

        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)


    def readSpeed(self):
        """ To be filled"""
        matrixRL = np.array([[Gradio / 2, Gradio / 2], [Gradio / GL, -Gradio / GL]])
        [wl, wr] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                    math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]
        x = wl - self.WL
        y = wr - self.WR
        self.WL = wl
        self.WR = wr
        matrixW = np.array([[y/self.P], [x/self.P]])
        matrixVW = np.dot(matrixRL, matrixW)
        print("WR:", y/self.P, "WL:", x/self.P)
        print("V:", matrixVW[0, 0], "w:", matrixVW[1, 0])
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

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):  # , additional_params?):
        # TODO: revisar la parte del UPDATE
        """ To be filled ...  """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            sys.stdout.write("Update of odometry ...., X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))
            # print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            # Operations like += which involve a read and write are not atomic.
            # with self.x.get_lock():
            #    self.x.value+=1

            # to "lock" a whole set of operations, we can use a "mutex"
            # self.lock_odometry.acquire()
            # self.x.value+=1
            # self.y.value+=1
            # self.th.value+=1
            # self.lock_odometry.release()

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                # sys.stdout.write("Reading encoder values .... \n")
                # B = izq, C = der
                # [encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                #                        self.BP.get_motor_encoder(self.BP.PORT_C)]
                difS = 0
                difTH = 0

                self.lock_odometry.acquire()
                v, w = self.readSpeed()

                self.lock_odometry.release()

                if (w == 0): #Cambiar por w pequeo?
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


                # Meter datos en el LOG
                self.LOG.write(str(self.x.value) + ' ' + str(self.y.value)
                               + ' ' + str(self.th.value) + '\n')

            except IOError as error:
                # print(error)
                sys.stdout.write(error)

            # sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))

            # TODO: decide when to store a log with the updated odometry
            # save LOG

            ######## UPDATE UNTIL HERE with your code ########

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
        # self.BP.reset_all()
        
    def velAng(xBlob, yBlob, diamBlob):
        
        
        #TODO   xCentroCam se calcula empiricamente
        xCentroCam = 
        
        d = xCemtroCam - xBlob
        #TODO   b se calcula empiricamente (distancia entre robot y pelota)
        b = 
        
        w = math.arcsin(d/b)
        
        return math.radians(w)
        
    def velLin():
        
        return v

    def trackObject(self, colorRangeMin, colorRangeMax, areaObjetivo):
        """ Esta funcion persigue la pelota roja hasta una posicion objetivo """
        finished = False
        targetFound = False
        targetPositionReached = False
        palante = False
              
        # Elegimos el umbral de rojo en HSV
        redMin1 = (170,100,20)
        redMax1 = (179,255,255)
        # Elegimos el segundo umbral de rojo en HSV
        redMin2 = (0,100,20)
        redMax2 = (8,255,255)
        
        detector = camInit()
        
        captura = cv2.VideoCapture(0)
        
        while not finished:  
             _, img = captura.read()
    
            img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
            # hacemos la mask y filtramos en la original
            mask1 = cv2.inRange(img_HSV, redMin1, redMax1)
            mask2 = cv2.inRange(img_HSV, redMin2, redMax2)
            mask_red = mask1 + mask2
            
            # detector finds "dark" blobs by default, so invert image for results with same detector
            keypoints_red = detector.detect(255-mask_red)
            
            # documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
            #"x":the x coordinate of each blob in the image.
            #"y":the y coordinate of each blob in the image.
            #"size":the diameter of the circle containing the blob.
            for kp in keypoints_red:
            	print (kp.pt[0], kp.pt[1], kp.size)
            #cv2.waitKey(0)    
                
            while not targetPositionReached:
                # 2. decide v and w for the robot to get closer to target position
                
                #hacer una funcion para calcular la w
                w = velAng(kp.pt[0], kp.pt[1], kp.size)
                if w != 0
                    #setSpeed w
                else
                    #hacer una funcion para calcular la v (todavia por mirar)
                    # Posible sol: establecer velocidad fija y dejar que el
                    # robot avance mientras el while comprueba la posicion
                    #si !palante
                    # setSpeed v
                    # palante = true
                
                #si la posicion del blob es la deseada por target y targetSize
                #entonces, position reached y finished
                if areaObjetivo <= area
                    targetPositionReached = True
                    finished = True
                    captura.release()
                
        return finished
    
    
    def catch(self):
        # decide the strategy to catch the ball once you have reached the target
        position
