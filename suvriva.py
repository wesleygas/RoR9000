#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
mini = [10, 0]
def GonnaCrash(mini):
	return mini[0]<0.3
def Dont(dire):
	if (dire==0):
		velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.3)
	else:
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, dire*2))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.1)
	return 'em_frente'


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



def main():
	global mini
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	if (mini[1] < 360 and mini[1] > 320) or (mini[1] < 40 and mini[1] > 0):
		#Dont(0)
		return 'do_lado_esquerdo'
	if (mini[1] < 360 and mini[1] > 288):
		#Dont(1)
		return 'do_lado_direito'
	elif (mini[1] < 72 and mini[1] > 0):
		#Dont(-1)
		return 'em_frente'
	return 'OK'
def normal():
	global mini
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	if GonnaCrash(mini):
		return 'algo_errado'
	else:
		velocidade = Twist(Vector3(2, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		return 'OK'
