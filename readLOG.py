# -*- coding: utf-8 -*-
"""
Created on Thu Mar 11 15:35:22 2021

@author: patri
"""


import numpy as np
import math
import matplotlib.pyplot as plt

# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje,c,tamano):
  if tamano=='p':
    largo=0.1
    corto=0.05
    descentre=0.01
  else:
    largo=0.5
    corto=0.25
    descentre=0.05

  trasera_dcha=np.array([-largo,-corto,1])
  trasera_izda=np.array([-largo,corto,1])
  delantera_dcha=np.array([largo,-corto,1])
  delantera_izda=np.array([largo,corto,1])
  frontal_robot=np.array([largo,0,1])
  tita=math.radians(loc_eje[2])
  Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
             [np.sin(tita), np.cos(tita), loc_eje[1]],
              [0,        0 ,        1]])
  Hec=np.array([[1,0,descentre],
              [0,1,0],
              [0,0,1]])
  extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
  robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
  plt.plot(robot[0,:], robot[1,:], c)
  
#def readLOG():
with open('LOG.txt') as f:
    lines = f.readlines()
    x = [line.split()[0] for line in lines]
    y = [line.split()[1] for line in lines]
    th = [line.split()[2] for line in lines]
    
    for i in range(len(lines)):
        loc_eje = [float(x[i]),float(y[i]),float(th[i])] # coordenadas
        print(loc_eje)
        
        dibrobot(loc_eje, 'r', 'p')