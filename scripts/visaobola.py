#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
import os
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO


class VisaoMissao2:
    def __init__(self):
        rospy.init_node('missao2_visao', anonymous=True)
        
        self.erro_pub = rospy.Publisher("/visao/erro_alvo", Point, queue_size=1)
        self.erro_mangueira_pub = rospy.Publisher("/visao/erro_mangueira", Point, queue_size=1)
        self.orientacao_pub = rospy.Publisher("/visao/mangueira_orientacao", Point, queue_size=1)
        self.base_pub = rospy.Publisher("/visao/erro_base", Point, queue_size=1)
        self.img_pub = rospy.Publisher("/visao/camera_anotada", Image, queue_size=1)
        
        rospy.loginfo("Carregando modelo YOLO oficial...")
        self.model = YOLO(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ai_models/pesooficialhang.pt"))
        
        # =========================================================
        # CONFIGURAÇÕES (sem conversao metros-pixels — apenas coeficientes)
        # =========================================================
        # Preenchidos a partir do primeiro frame em processar_visao()
        self.tela_w = None
        self.tela_h = None
        self.centro_x = None
        self.centro_y = None

        # Offset do gancho em relacao ao centro da camera, expresso
        # como fracao da meia-altura da tela (mesmo espaco dos erros
        # normalizados publicados). Borda inferior = +1.0, centro = 0.0.
        self.offset_gancho_coef = 0.3

        # Raio maximo de aceitacao da bola, como fracao da largura da tela.
        self.limite_aceitacao_frac = 3.0 / 5.0
        self.limite_aceitacao_pixels = None  # calculado a partir do frame
        # =========================================================

        self.bridge = CvBridge()
        self.frame_atual = None
        rospy.Subscriber("/webcam/image_raw", Image, self.camera_cb)

    def camera_cb(self, data):
        try:
            self.frame_atual = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError: pass

    def processar_visao(self):
        rate = rospy.Rate(3) 

        while self.frame_atual is None:
            rate.sleep()

        rospy.loginfo("Primeiro frame recebido")

        h_img, w_img = self.frame_atual.shape[:2]
        self.tela_h = float(h_img)
        self.tela_w = float(w_img)
        self.centro_x = self.tela_w / 2.0
        self.centro_y = self.tela_h / 2.0
        self.limite_aceitacao_pixels = int(self.tela_w * self.limite_aceitacao_frac)

        while not rospy.is_shutdown():
            if self.frame_atual is None:
                rate.sleep(); continue

            frame = self.frame_atual.copy()
            mask_frame = frame.copy()


            # Desenha um círculo na tela pra você ver qual é a zona de captura!
            cv2.circle(frame, (int(self.centro_x), int(self.centro_y)), self.limite_aceitacao_pixels, (255, 255, 255), 1)

            # ==========================================
            # 1. BOLA (Confiança 0.90)
            # ==========================================
            resultados = self.model.predict(frame, conf=0.50, verbose=False)
            melhor_box = None
            maior_confianca = 0.0

            for box in resultados[0].boxes:
                conf = float(box.conf[0])
                if conf > maior_confianca:
                    maior_confianca = conf
                    melhor_box = box

            if melhor_box is not None:
                x1, y1, x2, y2 = map(int, melhor_box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                
                # Distância em pixels do centro da tela
                erro_px_x = cx - self.centro_x
                erro_px_y = self.centro_y - cy 
                distancia_px = math.hypot(erro_px_x, erro_px_y)
                
                # SÓ AVISA O DRONE SE ESTIVER DENTRO DA ZONA PERMITIDA
                if distancia_px < self.limite_aceitacao_pixels:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2) # Verde = Aceito

                    # Erros normalizados em [-1, +1] (borda da tela = ±1)
                    erro_frente_bola = erro_px_y / (self.tela_h / 2.0)
                    erro_lado_bola   = erro_px_x / (self.tela_w / 2.0)
                    self.erro_pub.publish(Point(x=erro_frente_bola, y=erro_lado_bola, z=0.0)) 
                else:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2) # Vermelho = Ignorado
                
                # A Máscara preta apaga a bola independente se foi aceita ou não
                cv2.rectangle(mask_frame, (max(0, x1-10), max(0, y1-10)), 
                                          (min(int(self.tela_w), x2+10), min(int(self.tela_h), y2+10)), (0, 0, 0), -1)

            # ==========================================
            # 2. MANGUEIRA (Com o Offset de 15cm)
            # ==========================================
            hsv = cv2.cvtColor(mask_frame, cv2.COLOR_BGR2HSV)
            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])),
                cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
            )

            kernel = np.ones((5,5), np.uint8)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

            contornos, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contornos:
                maior = max(contornos, key=cv2.contourArea)
                if cv2.contourArea(maior) > 1000: 
                    [vx, vy, x, y] = cv2.fitLine(maior, cv2.DIST_L2, 0, 0.01, 0.01)
                    vx_f, vy_f, x_f, y_f = float(vx[0]), float(vy[0]), float(x[0]), float(y[0])
                    
                    # Normalizar para [0, pi): linha nao tem direcao, evita saltos de
                    # pi entre frames (cv2.fitLine pode inverter o vetor diretor).
                    angulo_radianos = math.atan2(vy_f, vx_f) % (math.pi / 2)
                    self.orientacao_pub.publish(Point(x=0.0, y=0.0, z=angulo_radianos))
                    
                    erro_px_x_m = x_f - self.centro_x
                    erro_px_y_m = self.centro_y - y_f

                    # Erros normalizados em [-1, +1], com offset do gancho subtraido
                    erro_frente_mang = (erro_px_y_m / (self.tela_h / 2.0)) - self.offset_gancho_coef
                    erro_lado_mang   = erro_px_x_m / (self.tela_w / 2.0)

                    self.erro_mangueira_pub.publish(Point(x=erro_frente_mang, y=erro_lado_mang, z=0.0))

                    p1 = (int(x_f - 1000 * vx_f), int(y_f - 1000 * vy_f))
                    p2 = (int(x_f + 1000 * vx_f), int(y_f + 1000 * vy_f))
                    cv2.line(frame, p1, p2, (0, 255, 0), 3)

                    alvo_y = int(self.centro_y + self.offset_gancho_coef * (self.tela_h / 2.0))
                    cv2.drawMarker(frame, (int(self.centro_x), alvo_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

            # ==========================================
            # 3. BASE AZUL (deteccao por cor + circularidade)
            # ==========================================
            hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_blue = cv2.inRange(hsv_full, np.array([100, 100, 50]), np.array([130, 255, 255]))
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

            cont_base, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            melhor_base = None
            melhor_area = 0
            for c in cont_base:
                area = cv2.contourArea(c)
                if area < 500:
                    continue
                perim = cv2.arcLength(c, True)
                if perim <= 0:
                    continue
                # circularidade: 1.0 = circulo perfeito, 0.785 = quadrado, <0.5 = alongado
                circ = 4 * math.pi * area / (perim * perim)
                if circ < 0.5:
                    continue
                if area > melhor_area:
                    melhor_area = area
                    melhor_base = c

            if melhor_base is not None:
                M = cv2.moments(melhor_base)
                if M["m00"] > 0:
                    bcx = int(M["m10"] / M["m00"])
                    bcy = int(M["m01"] / M["m00"])
                    # Mesma convencao das outras publicacoes: normalizado [-1,+1]
                    erro_base_frente = (self.centro_y - bcy) / (self.tela_h / 2.0)
                    erro_base_lado   = (bcx - self.centro_x) / (self.tela_w / 2.0)
                    self.base_pub.publish(Point(x=erro_base_frente, y=erro_base_lado, z=0.0))
                    cv2.drawContours(frame, [melhor_base], -1, (255, 0, 0), 2)
                    cv2.circle(frame, (bcx, bcy), 5, (255, 0, 0), -1)

            try:
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError: pass
                

if __name__ == '__main__':
    try:
        VisaoMissao2().processar_visao()
    except rospy.ROSInterruptException: pass
