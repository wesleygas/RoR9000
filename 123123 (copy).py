#! /usr/bin/env python
# -*- coding:utf-8 -*-




import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import transformations
import math

mini = [10, 0]
velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
angulo = 0
bateu = False
desvia = False


def leu_imu(dado):
	global angulo
	global bateu
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))
	if dado.linear_acceleration.x >= 2:
		print(dado.linear_acceleration.x)
	if dado.linear_acceleration.x >= 2.7:
		bateu = True
		angulo =math.degrees(math.atan2(dado.linear_acceleration.x , dado.linear_acceleration.y))
		
def tempo_de_batida(t = None):
	global tmp
	if t == None:
		if float(tmp - rospy.get_rostime().secs )<= 0:
			print ("1")
			return False
	else:
		print("12")exit
		tmp = rospy.get_rostime().secs
		rospy.sleep(t)
		return True

def Bateu(angulo):
	#velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
	#velocidade_saida.publish(velocidade)
	#rospy.sleep(0.5)
	if angulo>=100:
		velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 2))
		velocidade_saida.publish(velocidade)
		tempo_de_batida(1.5)
	elif angulo >80 and angulo < 100:
		velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 2))
		velocidade_saida.publish(velocidade)
		tempo_de_batida(2)

	elif angulo <=80:
		velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, -2))
		velocidade_saida.publish(velocidade)
		tempo_de_batida(1.5)
	

	
def GonnaCrash(mini):
	return mini[0]<0.3

def Dont(dire):
	if (dire==0):
		velocidade = Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.3)
	velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, dire*2))
	velocidade_saida.publish(velocidade)
	rospy.sleep(0.3)


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




if __name__=="__main__":

	rospy.init_node("le_scan")
	tmp = rospy.get_rostime().secs

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	recebe_scan2 = rospy.Subscriber("/imu", Imu, leu_imu, queue_size =1)
	#print("titi", LaserScan)



	while not rospy.is_shutdown():
		print(bateu)
		if GonnaCrash(mini):
			if bateu:
				print("batico")
				Bateu(angulo)
				bateu = False
			elif not tempo_de_batida():
				if (mini[1] <= 360 and mini[1] > 320) or (mini[1] < 40 and mini[1] >= 0):
					Dont(0)
				if (mini[1] <= 360 and mini[1] > 288):
					Dont(1)
				elif (mini[1] < 72 and mini[1] >= 0):
					Dont(-1)
			#print("ganna",mini)
		
		else:
			#print("ginna",mini)
			velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.1)
