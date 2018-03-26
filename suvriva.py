#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

mini = 0.5
velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

def GonnaCrash(mini):
	return mini<0.18
def Dont():
	global velocidade
	velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
	
def scaneou(dado):
	global mini
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	mini = dado.range_max
	#print(np.array(dado.ranges).round(decimals=2))
	for i in np.array(dado.ranges).round(decimals=2):
		if i >= dado.range_min:
			if mini > i:
				mini = i
	#print(mini)
	
	
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

	


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	print("titi", LaserScan)



	while not rospy.is_shutdown():
		print("Oeee", mini)
		if GonnaCrash(mini):
			Dont() 
			print("ganna",mini)
		else:
			print("ginna",mini)
			velocidade = Twist(Vector3(2, 0, 0), Vector3(0, 0, 2))
		velocidade_saida.publish(velocidade)
		rospy.sleep(2)


