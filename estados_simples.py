#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
import suvriva


limite = 200
ang_speed = 0.2

## Classes - estados
"""
Classe simples - estado que gira até o limite depois termina
"""
class Girando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['fim', 'girando'])
		self.numero_voltas = 0
	def execute(self, userdata):
		self.numero_voltas +=1
		if self.numero_voltas < limite:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			return 'girando'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fim'
class Batendo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['em_frente','OK', 'do_lado_esquerdo', 'do_lado_direito', 'em_frente'])
	def execute(self, userdata):
		return suvriva.main()
class Andando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['OK', 'algo_errado'])
	def execute(self, userdata):
		return suvriva.normal()

class GIRAE(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['em_frente'])
	def execute(self, userdata):
		suvriva.Dont(-1)
		return 'em_frente'

class GIRAD(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['em_frente'])
	def execute(self, userdata):
		suvriva.Dont(1)
		return 'em_frente'

class RE(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['em_frente'])
	def execute(self, userdata):
		suvriva.Dont(0)
		return 'em_frente'


# main
def main():
	global velocidade_saida
	rospy.init_node('unico_estado')

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Cria uma máquina de estados
	sm = smach.StateMachine(outcomes=['fim_geral'])

	# Preenche a Smach com os estados
	with sm:
		smach.StateMachine.add('ANDANDO', Andando(),
			transitions={'OK': 'ANDANDO',
			'algo_errado':'PERIGO'})
		smach.StateMachine.add('PERIGO', Batendo(),
			transitions={'em_frente': 'RE',
			'OK': 'ANDANDO',
			'do_lado_esquerdo':'GIRAD',
			'do_lado_direito':'GIRAE',
			'em_frente':'PERIGO'})
		smach.StateMachine.add('GIRAE', GIRAE(),
			transitions={'em_frente': 'PERIGO'})
		smach.StateMachine.add('GIRAD', GIRAD(),
			transitions={'em_frente': 'PERIGO'})
		smach.StateMachine.add('RE', RE(),
			transitions={'em_frente': 'PERIGO'})

	# Executa a máquina de estados
	outcome = sm.execute()

	print("Execute finished")


if __name__ == '__main__':
	print("Main")
	main()
