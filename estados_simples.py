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
from sensor_msgs.msg import LaserScan
import smach
import smach_ros
import su


vel = 0.3


limite = 200
ang_speed = 0.2
mini = [10, 0]

## Classes - estados
"""
Classe simples - estado que gira até o limite depois termina
"""
def GonnaCrash(mini):
	return mini[0]<0.4

def scaneou(dado):
	global mini
	mini = [dado.range_max, 0]
	lelescan=np.array(dado.ranges).round(decimals=2)
	for i in range(len(lelescan)):
		if lelescan[i] >= dado.range_min:
			pass
			if mini[0] > lelescan[i]:
				mini = [lelescan[i],i]
	#print (len(lelescan))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

def normal():
	global mini
	global vel
	if GonnaCrash(mini):
		return 'algo_errado'
	else:
		velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
		print("frente")
		velocidade_saida.publish(velocidade)
		return 'OK'







class Girando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['fim', 'girando'])
		self.numero_voltas = 0
	def execute(self, userdata):
		self.numero_voltas +=1
		if self.numero_voltas < limite:
			#vel = Tiwist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			#velocidade_saida.publish(vel)
			return 'girando'
		else:
			#vel = Tiwist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			#velocidade_saida.publish(vel)
			return 'fim'

class Batendo(smach.State):
	global velocidade_saida
	def __init__(self):
		smach.State.__init__(self, outcomes=['em_frente','OK'])
	def execute(self, userdata):
		return su.rodatudo(mini,velocidade_saida)
class Procurando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Nop', 'algo_errado', 'achei'])
	def execute(self, userdata):
		return normal()



# main
def main():
	global velocidade_saida
	global mini
	rospy.init_node('unico_estado')

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	# Cria uma máquina de estados
	sm = smach.StateMachine(outcomes=['fim_geral'])

	# Preenche a Smach com os estados
	with sm:
		smach.StateMachine.add('PROCURANDO', Procurando(),
			transitions={'Nop': 'PROCURANDO',
			'algo_errado':'PERIGO',
			'achei','ACHADO'})
		smach.StateMachine.add('PERIGO', Batendo(),
			transitions={'OK': 'PROCURANDO',
			'em_frente':'PERIGO'})
		smach.StateMachine.add('ACHADO', Achado(),
			transitions={})

	# Executa a máquina de estados
	outcome = sm.execute()

	#vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
	#velocidade_saida.publish(vel)

	print("Execute finished")


if __name__ == '__main__':
	print("Main")
	main()
