#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import visao_module
import mobilenet_simples
from math import pi
from std_msgs.msg import UInt8


import cormodule


bridge = CvBridge()

cv_image = None


#definições da camera
centro = [320, 240]
mediax = 0
media_cor = []
dist = None

atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
viu_cat = False

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

def scaneou(dado):
    global dist
    dist = list(dado.ranges)

batida= None
def bater(data):
    global batida
    batida = data.data
    print ("BATIDA")
    print(batida)
    return batida

def proximidade(dist):
    esquerda = []
    direita = []
    if dist is not None:
        for f in range(len(dist)):
            if f <= 90:
                if dist[f] ==float("Inf") or dist[f] ==0:
                    direita.append(10)
                else:
                    direita.append(dist[f])
            elif 360 >= f >=270:
                if dist[f] == float("Inf") or dist[f] == 0:
                    esquerda.append(10)
                else:
                    esquerda.append(dist[f])
        if min (esquerda) < 0.3:
            return "Esquerda"
        elif min (direita) < 0.3:
            return "Direita"
        elif min(direita) < 0.3 and min(esquerda) < 0.3:
            return "Direita"
        else:
            return "Normal"
v = 0.14  
w = (pi/10)
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    #print("frame")
    global cv_image
    global centro
    global media_cor
    global viu_cat
    global mediax
    global distancia

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    #print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, imagem, resultados =  visao_module.processa(cv_image)
        media_cor, centro, area =  cormodule.identifica_cor(cv_image)
        #print (centro)
        #print (resultados)
        #print (type(resultados))
        depois = time.clock()
        cv2.imshow("Camera", cv_image)

        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado
            if r[0] == "cat":
                viu_cat = True
                pos_i = r[2]
                pos_f = r[3]
                x_i = pos_i[0]
                x_f= pos_f[0]
                mediax = (x_i+x_f)/2
                #print (pos_i)
                #print (pos_f)
                #print (x_i)
                #print (x_f)
                #print (centro)
                #print (resultados)
                #print (type(resultados))
                #print (mediax)

        depois = time.clock()
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("gato")

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_batida = rospy.Subscriber("/bumper", UInt8, bater)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


    topico_imagem = "/kamera"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    try:

        while not rospy.is_shutdown():

            #if dist < 0.5:
                #vel = Twist(Vector3(0,0,0), Vector3(0,0,pi))
                #velocidade_saida.publish(vel)
                #rospy.sleep(1)
            perigo = proximidade(dist)
            if perigo == "Normal":
                if batida == 1:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
                    vel = Twist(Vector3(-v/5,0,0), Vector3(0,0,-w*4))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1.5)
                    batida = 0
                elif batida == 2:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
                    vel = Twist(Vector3(-v/5,0,0), Vector3(0,0,4*w))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1.5)
                    batida = 0
                elif batida == 3:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
                    vel = Twist(Vector3(2*v,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1.5)
                    batida = 0
                elif batida == 4:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
                    vel = Twist(Vector3(2*v,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(1.5)
                    batida = 0

                elif batida is None or batida == 0: #and dist > 0.5:
                    if viu_cat:
                        if mediax-centro[0]>30:
                            vel = Twist(Vector3(-0.8,0,0), Vector3(0,0,-0.8))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.8)
                            viu_cat = False
                        elif centro[0]-mediax>30:
                            vel = Twist(Vector3(-0.8,0,0), Vector3(0,0,0.8))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.8)
                            viu_cat = False
                        elif mediax-centro[0]<30 or centro[0]-mediax< 30:
                            vel = Twist(Vector3(-0.8,0,0), Vector3(0,0,0))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.8)
                            viu_cat = False

                    elif len(media_cor) != 0 and len(centro) != 0:
                        if media_cor[0] > centro[1]:
                            vel = Twist(Vector3(0.08,0,0), Vector3(0,0,-0.25))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)
                        elif media_cor[0] < centro[1]:
                            vel = Twist(Vector3(0.08,0,0), Vector3(0,0,0.25))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)
                        else:
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)
            elif perigo == "Direita":
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.4))
                velocidade_saida.publish(vel)
                rospy.sleep(0.4)

            elif perigo == "Esquerda":
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.4))
                velocidade_saida.publish(vel)
                rospy.sleep(0.4)



    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")