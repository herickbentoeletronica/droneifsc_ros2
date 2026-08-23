#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
import os
import threading
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped, Point
from ultralytics import YOLO

class Missao1Eletroquad:
    def __init__(self):
        rospy.init_node('visao_missao_node', anonymous=False)
        self.bridge = CvBridge()

        # Estado do Drone
        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, 0.0
        self.posicao_recebida = False
        self.comando_pouso_enviado = False

        # Memória da Missão
        self.gabarito_encontrado = False
        self.gabarito_forma = None
        self.gabarito_id = None
        self.bases_encontradas = []

        # Memória para rastreio contínuo do alvo autorizado
        self.alvo_forma_confirmada = None
        self.alvo_numero_confirmado = None

        # Controle de Frames
        self.latest_frame = None
        self.latest_header = None
        self.frame_lock = threading.Lock()
        self.rate = rospy.Rate(3) 

        self._configurar_modelos()
        self.publish_compressed = rospy.get_param('~publish_compressed', True)

        # ROS Subscribers e Publishers
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_callback)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
	    
        self.compressed_img_pub = rospy.Publisher('/visao/resultado_yolo_aruco/compressed', CompressedImage, queue_size=1) if self.publish_compressed else None
        self.image_pub = rospy.Publisher("/visao/resultado_yolo_aruco", Image, queue_size=1)
        self.alvo_pub = rospy.Publisher("/visao/alvo_pouso", Point, queue_size=10)
        self.erro_pub = rospy.Publisher("/visao/erro_alvo", Point, queue_size=10)

        rospy.on_shutdown(self._forcar_saida)
        rospy.loginfo("No de Visao Iniciado")

    def _forcar_saida(self):
        rospy.loginfo("Saida forcada para liberar threads do YOLO/torch")
        os._exit(0)

    def _configurar_modelos(self):
        try:
            #self.model = YOLO(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ai_models/best_ncnn_model"))
            self.model = YOLO(os.path.join(os.path.dirname(os.path.abspath(__file__)), "pesocomcameradrone.pt"))
            rospy.loginfo("Modelo YOLO carregado com sucesso!")
        except Exception as e:
            rospy.logerr(f"Erro ao carregar YOLO: {e}")
            self.model = None

        # Configuração do ArUco (OpenCV)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_parametros = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_parametros)
        rospy.loginfo("Aruco detector carregado")


    def pos_callback(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_z = msg.pose.position.z
        self.posicao_recebida = True

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            with self.frame_lock:
                self.latest_frame = frame
                self.latest_header = data.header
        except CvBridgeError as e:
            rospy.logerr(e)


    def enviar_comando_pouso(self, x, y):
        if self.comando_pouso_enviado:
            return

        msg_alvo = Point(x=x, y=y, z=0.0)
        self.alvo_pub.publish(msg_alvo)
        self.comando_pouso_enviado = True
        rospy.logwarn(f"COMANDO DE POUSO ENVIADO PARA X:{x:.2f}, Y:{y:.2f}")

    def _avaliar_condicao_pouso(self, forma_base, numero_base, x, y, nome_base_completo):
        if not self.gabarito_encontrado:
            return False

        if (forma_base == self.gabarito_forma) and (self.gabarito_id % numero_base == 0):
            rospy.loginfo("="*50)
            rospy.loginfo(f"POUSO AUTORIZADO! BASE CORRETA: {nome_base_completo.upper()}")
            rospy.loginfo(f"   Coordenada Global: X:{x:.2f}, Y:{y:.2f}")
            rospy.loginfo("="*50)

            self.alvo_forma_confirmada = forma_base
            self.alvo_numero_confirmado = numero_base
            self.enviar_comando_pouso(x, y)
            return True
        return False

    def _processar_novo_gabarito(self, forma, aruco_id):
        if self.gabarito_encontrado:
            return

        self.gabarito_encontrado = True
        self.gabarito_forma = forma
        self.gabarito_id = aruco_id

        rospy.loginfo("*"*50)
        rospy.loginfo(f"GABARITO MEMORIZADO! Forma: {forma.upper()} | ID: {aruco_id}")
        rospy.loginfo("*"*50)

        for base in self.bases_encontradas:
            self._avaliar_condicao_pouso(base['forma'], base['numero'], base['x'], base['y'], base['nome'])

    def _processar_nova_base(self, nome_completo, forma, numero):
        if any(b['nome'] == nome_completo for b in self.bases_encontradas):
            return

        nova_base = {'nome': nome_completo, 'forma': forma, 'numero': numero, 'x': self.drone_x, 'y': self.drone_y}
        self.bases_encontradas.append(nova_base)
        rospy.loginfo(f"BASE MAPEADA: {nome_completo.upper()} em X:{self.drone_x:.2f}, Y:{self.drone_y:.2f}")

        self._avaliar_condicao_pouso(forma, numero, self.drone_x, self.drone_y, nome_completo)

    def _fazer_fusao_dados(self, deteccoes_yolo, deteccoes_aruco):
        TOLERANCIA_PIXELS = 160
        
        formas = [d for d in deteccoes_yolo if d['nome'] in ['estrela', 'hexagono', 'triangulo']]
        numeros_brutos = [d for d in deteccoes_yolo if d['nome'] in ['num_3', 'num_4', 'num_5']]

        #filtro pra ignorar os numeros em aruco
        numeros_validados = []
        for num in numeros_brutos:
            confundiu_com_aruco = False
            for aruco in deteccoes_aruco:
                dist = math.hypot(num['cx'] - aruco['cx'], num['cy'] - aruco['cy'])
                if dist < TOLERANCIA_PIXELS:
                    confundiu_com_aruco = True
                    break
            if not confundiu_com_aruco:
                numeros_validados.append(num)

        bases_montadas_neste_frame = []

        #Associacao geometrica
        for forma in formas:
            forma_eh_gabarito = False
            
            for aruco in deteccoes_aruco:
                dist = math.hypot(forma['cx'] - aruco['cx'], forma['cy'] - aruco['cy'])
                if dist < TOLERANCIA_PIXELS:
                    self._processar_novo_gabarito(forma['nome'], aruco['id'])
                    forma_eh_gabarito = True
                    break 

            if forma_eh_gabarito:
                continue 

            for num in numeros_validados:
                dist = math.hypot(forma['cx'] - num['cx'], forma['cy'] - num['cy'])
                if dist < TOLERANCIA_PIXELS:
                    valor_numero = int(num['nome'].replace('num_', ''))
                    nome_base_completo = f"{forma['nome']}_{valor_numero}"

                    self._processar_nova_base(nome_base_completo, forma['nome'], valor_numero)
                    
                    bases_montadas_neste_frame.append({
                        'forma': forma['nome'],
                        'numero': valor_numero,
                        'cx': forma['cx'],
                        'cy': forma['cy']
                    })
                    break 

        return bases_montadas_neste_frame

    def rodar(self):
        while not rospy.is_shutdown():
            try:
                with self.frame_lock:
                    if self.latest_frame is None:
                        self.rate.sleep()
                        continue
                    frame = self.latest_frame.copy()
                    header_atual = self.latest_header
                    self.latest_frame = None

                frame_anotado = frame.copy()
                deteccoes_yolo = []
                deteccoes_aruco = []

                #roda yolo
                if self.model is not None:
                    resultados = self.model.predict(source=frame, conf=0.65, verbose=False)
                    frame_anotado = resultados[0].plot()

                    for caixa in resultados[0].boxes:
                        nome = self.model.names[int(caixa.cls[0])]
                        coords = caixa.xyxy[0].tolist()
                        cx = int((coords[0] + coords[2]) / 2)
                        cy = int((coords[1] + coords[3]) / 2)
                        deteccoes_yolo.append({'nome': nome, 'cx': cx, 'cy': cy})

                # 2.roda opencv
                corners, ids, _ = self.aruco_detector.detectMarkers(frame)

                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame_anotado, corners, ids)
                    for i, cantos in enumerate(corners):
                        cx, cy = np.mean(cantos[0], axis=0).astype(int)
                        deteccoes_aruco.append({'id': int(ids[i][0]), 'cx': cx, 'cy': cy})

                #processafusao
                bases_no_frame = self._fazer_fusao_dados(deteccoes_yolo, deteccoes_aruco)

                # 4. Rastreio Servo-Visual (Apenas se já autorizou o pouso)
                # Publica erro como pixel NORMALIZADO [-1..1] do centro do frame (igual gauge_reader).
                # zigzagnode aplica GAIN e converte para metros body.
                if self.alvo_forma_confirmada is not None and self.alvo_numero_confirmado is not None:
                    for base in bases_no_frame:
                        if base['forma'] == self.alvo_forma_confirmada and base['numero'] == self.alvo_numero_confirmado:
                            h_img, w_img = frame.shape[:2]
                            dx_norm = -(base['cy'] - h_img / 2.0) / (h_img / 2.0)
                            dy_norm = (base['cx'] - w_img / 2.0) / (w_img / 2.0)

                            msg_erro = Point()
                            msg_erro.x = float(dx_norm)
                            msg_erro.y = float(dy_norm)
                            msg_erro.z = 0.0   # stub
                            self.erro_pub.publish(msg_erro)

                            cv2.circle(frame_anotado, (base['cx'], base['cy']), 15, (0, 255, 0), -1)
                            cv2.putText(frame_anotado, "LOCKED", (base['cx']-30, base['cy']-25),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            break

                #falar
                try:
                    msg_publicar = self.bridge.cv2_to_imgmsg(frame_anotado, "bgr8")
                    msg_publicar.header = header_atual
                    self.image_pub.publish(msg_publicar)
                    if self.compressed_img_pub is not None:
                        _, jpeg = cv2.imencode('.jpg', frame_anotado, [cv2.IMWRITE_JPEG_QUALITY, 10])
                        msg = CompressedImage()
                        msg.header.stamp = rospy.Time.now()
                        msg.format = 'jpeg'
                        msg.data = jpeg.tobytes()
                        self.compressed_img_pub.publish(msg)

                except CvBridgeError as e:
                    rospy.logerr(e)

            except Exception as e:
                rospy.logerr(f"Erro no loop de visao: {e}", exc_info=True)

            self.rate.sleep()

if __name__ == '__main__':
    visao = Missao1Eletroquad()
    try:
        visao.rodar()
    except rospy.ROSInterruptException:
        pass
