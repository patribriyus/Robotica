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

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

# TODO: calcular unidades
Gradio = 0.027  # Radio ruedas motoras (m - ahora mismo)
# TODO: Medir distancia entre ruedas
GL = 0.137  # Distancia entre ruedas motoras (m - ahora mismo entre centros ruedas)


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        ######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters

        # Velocidad angular de las ruedas Right y Left
        # TODO: calcular en que unidades
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
        self.P = 1.0  # tiempo entre cada comprobacion de la odometria (thread)

    def setSpeed(self, v, w):
        """ To be filled - These is all dummy sample code """
        # TODO: revisar que es correcto
        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...

        matrixVW = np.array([[v], [w]])
        matrixRL = np.array([[1 / Gradio, GL / (2 * Gradio)], [1 / Gradio, -GL / (2 * Gradio)]])
        matrixW = np.dot(matrixRL, matrixVW)

        # Establecer velocidad a ambos motores a la vez
        # speedPower = v
        # BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        # TODO: Comprobar que el puerto B es para la rueda izq y C para der.
        # Set the motor target speed in degrees per second
        speedDPS_right = degrees(matrixW[0][0])
        speedDPS_left = degrees(matrixW[1][0])
        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)

    def readSpeed(self):
        """ To be filled"""
        matrixRL = np.array([[Gradio / 2, Gradio / 2], [Gradio / GL, -Gradio / GL]])
        [WL, WR] = [math.radians(self.BP.get_motor_encoder(self.BP.PORT_B)),
                    math.radians(self.BP.get_motor_encoder(self.BP.PORT_C))]
        matrixW = np.array([[WR], [WL]])
        matrixVW = np.dot(matrixRL, matrixW)

        return matrixVW[0, 0], math.radians(matrixVW[1, 0])

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
            sys.stdout.write("Dummy update of odometry ...., X=  %d, \
                Y=  %d, th=  %d \n" % (self.x.value, self.y.value, self.th.value))
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

                if (w == 0):
                    difS = v * self.P
                else:
                    difTH = w * self.P
                    difS = (v / w) * difTH

                self.x.value = difS * math.cos(self.th.value + difTH / 2)
                self.y.value = difS * math.sin(self.th.value + difTH / 2)
                self.th.value = difTH
                self.lock_odometry.release()

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

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        # self.BP.reset_all()
