#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header


import visao_module

from sklearn.linear_model import LinearRegression

bridge = CvBridge()

cv_image = None
ponto_medio = (0,0)
theta = 0.0
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
linear_regressor = LinearRegression()

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)


def processa_imagem(imagem): # CHECK
    '''
    recebe imagem e devolve o angulo da regressao com a horizontal
    '''

    # Filtrando amarelos:
    frame = imagem.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    low = (25, 50, 50)
    high = (35, 255, 255)
    mask = cv2.inRange(hsv, low, high)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.erode(mask,kernel,iterations = 1)
    #mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    #mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )
    cv2.imshow("Linhas amarelas", mask)


    #Thresh
    ret, thresh = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)

    

    #Encontrando os pontos brancos da mask para a regressão
    pontos_brancos = np.where(mask==255)
    lista_x = pontos_brancos[1]
    lista_y = pontos_brancos[0]
    
    print(lista_x)

    #linear_regressor = LinearRegression()  # create object for the class

    lista_x = np.array(lista_x)
    lista_x = lista_x.reshape(-1,1)
    lista_y = np.array(lista_y)
    lista_y = lista_y.reshape(-1,1)
    
    if len(lista_x) > 0 and len(lista_y) > 0:
        linear_regressor.fit(lista_y, lista_x)  # inveção do x e y para a regresão
            
        X = np.array([0, frame.shape[1]]).reshape(-1, 1)
        Y_pred = linear_regressor.predict(X)  # make predictions
        img_regres = cv2.line(frame, (int(Y_pred[0]),int(X[0])), (int(Y_pred[-1]),int(X[-1])), (0, 255, 0), thickness=3, lineType=8)


        # voltando com as coordenadas do openCV:
        X_cv = Y_pred
        Y_cv = X

        #Encontrando os contornos

        # encontrar centros de massa
        lista_x = [] # coordenadas x
        lista_y = [] # coordenadas y

        contornos, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        ### MAIOR CONTORNO
        maior = None
        maior_area = 0
        for c in contornos:
            area = cv2.contourArea(c)
            if area > maior_area:
                maior_area = area
                maior = c

            M = cv2.moments(maior)

            try:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                crosshair(frame, (cX,cY), size=10, color=(0, 0, 255))
            except:
                print("passou")

        


        #index_ponto_base_y = Y_pred.index([frame.shape[1]])
        #ponto_base_x = X[index_ponto_base_y]


        #crosshair(frame, (int(ponto_base_x),int(frame.shape[1])), size=10, color=(255, 255, 255))

        
        #pontos da curva
        X1 = X[0]
        X2 = X[-1]
        delta_X = X2 - X1
        
        medio_X = (X1+X2)/2

        Y1 = Y_pred[0]
        Y2 = Y_pred[-1]
        delta_Y = Y2 - Y1
        medio_Y = (Y1+Y2)/2
        ponto_medio = (medio_X, medio_Y)

        crosshair(frame, (int(medio_X),int(medio_Y)), size=10, color=(255, 255, 0))

        angulo_in = math.degrees(math.atan2(delta_X, delta_Y))

        if 180 > angulo_in > 90:
            angulo = angulo_in - 90
        elif angulo_in < 90:
            angulo = angulo_in + 90
        else:
            angulo = angulo_in
        print(angulo)
    cv2.imshow("regressão", frame)
    return angulo, ponto_medio


def percorrendo_pista(angulo, ponto_medio):        
    '''if 50 < angulo < 130: #se o angulo for entre 80 e 100, o robô vai reto
        frente = Twist(Vector3(0.25,0,0), Vector3(0,0,0))
        velocidade_saida.publish(frente)
    elif angulo < 50:
        direita = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
        velocidade_saida.publish(direita)
    elif angulo > 130:
        esquerda = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
        velocidade_saida.publish(esquerda)'''

    if ponto_medio[0] > 640/2+10:
        direita = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
        velocidade_saida.publish(direita)
    elif ponto_medio[0] < 640/2-10:
        esquerda = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
        velocidade_saida.publish(esquerda)
    else:
        frente = Twist(Vector3(0.25,0,0), Vector3(0,0,0))
        velocidade_saida.publish(frente)

    return None


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global theta
    global ponto_medio

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        # Esta logica do delay so' precisa ser usada com robo real e rede wifi 
        # serve para descartar imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass
        
        
        
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        theta, ponto_medio = processa_imagem(cv_image)




        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    #CRIAÇÃO DE ESTADOS:
    ANDANDO = 0


    try:
        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            
            #percorrendo_pista(theta, ponto_medio)
            #velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")