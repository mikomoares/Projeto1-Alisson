#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import UInt8

batida= None
def bater(data):
	global batida
	batida = data.data
	print(batida)
	return batida

v = 0.14  # Velocidade linear
w = (pi/10)  # Velocidade angular	


if __name__=="__main__":

	rospy.init_node("batida")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_batida = rospy.Subscriber("/bumper", UInt8, bater)


	while not rospy.is_shutdown():
		#print("Oeee")
		if batida is None or batida == 0:
			velocidade = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.1)
		elif batida == 1:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			vel = Twist(Vector3(-v/5,0,0), Vector3(0,0,-w))
			velocidade_saida.publish(vel)
			rospy.sleep(6)
			batida = 0
		elif batida == 2:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			vel = Twist(Vector3(-v/5,0,0), Vector3(0,0,w))
			velocidade_saida.publish(vel)
			rospy.sleep(6)
			batida = 0
		elif batida == 3:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			vel = Twist(Vector3(v/5,0,0), Vector3(0,0,-w))
			velocidade_saida.publish(vel)
			rospy.sleep(6)
			batida = 0
		elif batida == 4:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			vel = Twist(Vector3(v/5,0,0), Vector3(0,0,w))
			velocidade_saida.publish(vel)
			rospy.sleep(6)
			batida = 0
