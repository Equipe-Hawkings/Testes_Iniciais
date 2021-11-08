#!/usr/bin/env python

############################################################################
##                            Equipe Hawkings                             ##
############################################################################
## author: Felipe Gabriel                                                 ##
## last update(d/m/a): 01/11/2021                                         ##
############################################################################
## Codigo de teste de movimentacao, que utiliza o módulo motion_control.  ##
## Ele define tres posicoes para o drone ir, realizando um trajeto, e o   ##
## numero de ciclos, para realizar o trajetos uma certa quantidade de     ##
## vezes.                                                                 ##
############################################################################

from re import X
import rospy
import unittest
import math
import numpy as np
import time
from rospy.exceptions import ROSException
from std_msgs.msg import String, Header
from drone_control.motion_control import MotionControl
# Import tipagens msg e str

class Drone(MotionControl):
   def __init__(self):
      MotionControl.__init__(self)

   def trajeto(self, x, y, z, erro, msg_inicial = None, msg_final = None):
      if (msg_inicial is not None):
         rospy.loginfo(msg_inicial)
      while True:
         self.setpoint(x, y, z)
         if self.chegou(erro) == True:
            if (msg_inicial is not None):
               rospy.loginfo(msg_final)
            break
   
   def run(self):
      ciclos = int(input("Quantas voltas deseja realizar?: "))
      if ciclos != 0:
         contCiclos = 0
         for i in range (150):
            self.setpoint(0, 0, 0)
         self.armar()
         while not rospy.is_shutdown() and contCiclos < ciclos:
            erroTrajeto = 0.4
            erro_TakeOff_Land = 0.2
            if self.modoDeVoo.mode != "OFFBOARD":
               self.setModoDeVoo("OFFBOARD")
               if self.modoDeVoo.mode == "OFFBOARD":
                  self.trajeto(0, 0, 2, erro_TakeOff_Land, "Decolando...")
            else:
               self.trajeto(5, 2, 3, erroTrajeto, None, "Chegou à posição 1.")
               self.trajeto(3, 5, 5, erroTrajeto, None, "Chegou à posição 2.")
               self.trajeto(0, 0, 4, erroTrajeto, None, "Chegou à posição 3.")
               contCiclos += 1
         self.trajeto(0, 0, 2, erro_TakeOff_Land)
         self.pousar()
         rospy.loginfo("Missão encerrada!")
   
   def testeCV(self):
      for i in range (150):
            self.setpoint(0, 0, 0)
      self.armar()
      while not rospy.is_shutdown():
         if self.modoDeVoo.mode != "OFFBOARD":
            self.setModoDeVoo("OFFBOARD")
         else:
            self.trajeto(0, 0, 1, 0.3)


if __name__ == "__main__":
   try:
      rospy.init_node('mission', anonymous=True)
      Interprise = Drone()
      Interprise.run()
      #Interprise.testeCV()
   except rospy.ROSInterruptException:
      pass
