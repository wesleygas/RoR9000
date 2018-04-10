#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import transformations
import math
import time

def leu_imu(dado):
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))
	mensagem = """
	Tempo: {:}
	Orientação: {:.2f}, {:.2f}, {:.2f}
	Vel. angular: x {:.2f}, y {:.2f}, z {:.2f}\

	Aceleração linear:
	x: {:.2f}
	y: {:.2f}
	z: {:.2f}


""".format(dado.header.stamp, angulos[0], angulos[1], angulos[2], dado.angular_velocity.x, dado.angular_velocity.y, dado.angular_velocity.z, dado.linear_acceleration.x, dado.linear_acceleration.y, dado.linear_acceleration.z)
	#print(mensagem)
	if dado.linear_acceleration.x >= 2.8:

		angulo =math.degrees(math.atan2(dado.linear_acceleration.x , dado.linear_acceleration.y))
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

		#return "bateu_frente"
	
	
	else:
		velocidade = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		#return "OK"

		


if __name__=="__main__":

	rospy.init_node("le_imu")
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu, queue_size = 2)

	while not rospy.is_shutdown():
		print("Main loop")
		rospy.sleep(2)


