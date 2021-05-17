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
import cv2.aruco as aruco

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from termcolor import colored


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
largura_tela = 640
area = 0.0 # Variavel com a area do maior contorno
cX =0
cY =0
centro_x_creeper=0
centro_y_creeper=0
area = 0
x_bifurcacao1 = 10 #obtido pela odometria
y_bifurcacao1 = 10 #obtido pela odometria

x_bifurcacao2 = 10 #obtido pela odometria
y_bifurcacao2 = 10 #obtido pela odometria

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

topico_odom = "/odom"

# Apenas valores para inicializar odometria
x = -1000
y = -1000
z = -1000

def recebeu_leitura(dado):
    """
        Grava nas variáveis x,y,z a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
    """
    global x
    global y 
    global z 

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z

    print("odometria", x,y)

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
    global largura_tela 
    global x_bifurcacao1
    global x_bifurcacao2
    global y_bifurcacao1
    global y_bifurcacao2

    # Na bifurcacao: ele olha so para a direita 

    # ARUCO:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None:

        

        aruco.drawDetectedMarkers(frame, corners, ids)

        calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
        camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
        camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

        marker_size = 20


        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
        distancenp = np.linalg.norm(tvec)



        str_dist = "Dist aruco=%4.0f  dis.np=%4.0f"%(distance, distancenp)
        print(str_dist)

        for i in range(len(ids)):
                print('ID: {}'.format(ids[i]))
                print("Distancia = ",distance)
                for c in corners[i]:
                    for canto in c:
                        if ids[0] == 100 and (65 <= distance <= 85):
                            x_bifurcacao1 = x 
                            y_bifurcacao1 = y
                            print("Entrei aqui!")
                        if ids[0] == 200 and (45 <= distance <= 65):
                            x_bifurcacao2 = x
                            y_bifurcacao2 = y


                        print("Corner {}".format(canto))
        print("CORNER", corners[i])
    

    if (((x-x_bifurcacao1)**2 + (y-y_bifurcacao1)**2)**0.5 <= 0.6 )or (((x-x_bifurcacao2)**2 + (y-y_bifurcacao2)**2)**0.5 <= 0.6): #circulo que abrange o ponto
        largura_tela = 200
        print("Entrei aqui")
        crosshair(frame, (int(largura_tela/2),190), 3, (255,255,255))
    else:
        largura_tela = 640
    


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
        #img_regres = cv2.line(frame, (int(Y_pred[0]),int(X[0])), (int(Y_pred[-1]),int(X[-1])), (0, 255, 0), thickness=3, lineType=8)


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
                cv2.imshow("regressão", frame)
                return cX, cY
                
            except:
                cv2.imshow("regressão", frame)
                print("passou")

        #index_ponto_base_y = Y_pred.index([frame.shape[1]])
        #ponto_base_x = X[index_ponto_base_y]


        #crosshair(frame, (int(ponto_base_x),int(frame.shape[1])), size=10, color=(255, 255, 255))

        #pontos da curva

    else: # QUANDO O ROBO NÃO VE NADA AMARELO
        direita = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
        velocidade_saida.publish(direita)
        cv2.imshow("regressão", frame)
        a = colored('Estou rodando','red')
        print(a)

    return 0,0
    

def centraliza_crosshair(x_centro, y_centro):        

    # Loop principal: centraliza no centro do maior contorno amarelo
    global largura_tela

    if (largura_tela/2 - 20) < x_centro < (largura_tela/2 + 20):
        frente = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
        velocidade_saida.publish(frente)

    elif (largura_tela/2 - 20) > x_centro:
        direita = Twist(Vector3(0.05,0,0), Vector3(0,0,0.2))
        velocidade_saida.publish(direita)
    
    elif (largura_tela/2 + 20) < x_centro:
        esquerda = Twist(Vector3(0.05,0,0), Vector3(0,0,-0.2))
        velocidade_saida.publish(esquerda)


    return None

def encontra_creepers(imagem_in):

    imagem = imagem_in.copy() 
    hsv = cv2.cvtColor(imagem, cv2.COLOR_BGR2HSV)

    #### Segmentando cores

    hsv1_orange = (0, 250, 250)
    hsv2_orange = (20, 255, 255)

    hsv1_verde = (55, 150, 150)
    hsv2_verde = (80, 255, 255)

    hsv1_ciano = (80, 150, 150)
    hsv2_ciano = (100, 255, 255)

    ### Fazendo masks

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,4))
    
    mask_orange = cv2.inRange(hsv, hsv1_orange, hsv2_orange)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)

    mask_verde = cv2.inRange(hsv, hsv1_verde, hsv2_verde)
    mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_OPEN, kernel)

    mask_ciano = cv2.inRange(hsv, hsv1_ciano, hsv2_ciano)
    mask_ciano = cv2.morphologyEx(mask_ciano, cv2.MORPH_OPEN, kernel)

    cv2.imshow("mask vermelho", mask_orange)
    cv2.imshow("mask verde", mask_verde)
    cv2.imshow("mask ciano", mask_ciano)

    list_masks = [mask_orange, mask_ciano, mask_verde]

    for mask in list_masks:

        ret, thresh = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)
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
                crosshair(imagem, (cX,cY), size=10, color=(0, 0, 255))
                cv2.imshow("regressão", imagem)
                return cX, cY, area
                
            except:
                b = colored("caiu aqui", "green")
                print(b)

    return None, None, None

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global cY
    global cX
    global area
    global centro_x_creeper
    global centro_y_creeper

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
        cX, cY = processa_imagem(temp_image)
        centro_x_creeper, centro_y_creeper, area = encontra_creepers(temp_image)
        




        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber(topico_odom, Odometry , recebeu_leitura)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    #CRIAÇÃO DE ESTADOS:
    ANDANDO_PISTA = 0
    AVANCANDO_CREEPER = 1
    SEGURANDO_CREEPER = 2
    VOLTANDO_PISTA = 3
    ACHANDO_BASE = 4
    SOLTANDO_CREEPER = 5

    state = ANDANDO_PISTA

    def andando_pista():
        global cX
        global cY
        centraliza_crosshair(cX,cY) # percorre pista, centralizando no maior contorno amarelo
        return None

    def avancando_creeper():
        print('avancando creeper')
        global centro_x_creeper
        global centro_y_creeper
        centraliza_crosshair(centro_x_creeper, centro_y_creeper)
        return None

    def segurando_creeper():
        return None

    def voltando_pista():
        return None
    
    def achando_base():
        return None

    def soltando_creeper():
        return None

    def dispatch():
        global state
        global area
        if area!=None and centro_x_creeper!=None and centro_y_creeper!=None:
            state = AVANCANDO_CREEPER
        
        return None



    acoes = {
        ANDANDO_PISTA: andando_pista,
        AVANCANDO_CREEPER: avancando_creeper,
        SEGURANDO_CREEPER: segurando_creeper,
        VOLTANDO_PISTA:voltando_pista,
        ACHANDO_BASE: achando_base,
        SOLTANDO_CREEPER: soltando_creeper
        }

    try:
        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            dispatch()
            #percorrendo_pista(theta, ponto_medio)
            #velocidade_saida.publish(vel)
            acoes[state]()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print ("Ocorreu uma exceção com o rospy")