#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def GonnaCrash(mini):
	return mini[0]<0.4
def Dont(dire,vel):
	if (dire==0):
		velocidade = Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0))
		vel.publish(velocidade)
		rospy.sleep(0.3)
	velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, dire*2))
	vel.publish(velocidade)
	rospy.sleep(0.3)

def bateu(angulo):
	velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.5)
		
		if angulo>=100:
			velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 2))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1.5)

		elif angulo >80 and angulo < 100:
			velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 2))
			velocidade_saida.publish(velocidade)
			rospy.sleep(2)

		elif angulo <=80:
			velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, -2))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1.5)

		return "OK"
def rodatudo(mini,vel):
		if GonnaCrash(mini):
			if (mini[1] <= 360 and mini[1] > 320) or (mini[1] < 40 and mini[1] >= 0):
				Dont(0,vel)
			if (mini[1] <= 360 and mini[1] > 280):
				Dont(1,vel)
			elif (mini[1] < 80 and mini[1] >= 0):
				Dont(-1,vel)
			
			return "em_frente"
		else:
			return "OK"
