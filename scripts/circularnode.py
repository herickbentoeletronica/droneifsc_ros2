#!/usr/bin/env python3
import os
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

ALTURA_MISSAO = 3

class Missao2EletroQuad:
    def __init__(self):
        rospy.init_node('missao2_navegacao', anonymous=False)
        self.rate = rospy.Rate(10)

        self.estado = State()
        self.pose_atual = PoseStamped()
        
        # Dados da Bola (Fase 1)
        self.erro_x = 0.0 # Frente/Trás
        self.erro_y = 0.0 # Direita/Esquerda
        self.ultima_visao_bola = rospy.Time(0)

        # Dados da Mangueira (Fase 2)
        self.erro_m_x = 0.0
        self.erro_m_y = 0.0
        self.mangueira_yaw = 0.0
        self.ultima_visao_mangueira = rospy.Time(0)

        # Dados da Base Azul (Fase 3 - pouso)
        self.erro_base_x = 0.0
        self.erro_base_y = 0.0
        self.ultima_visao_base = rospy.Time(0)

        # -------------------------------------------------------------
        # NOVO PUBLICADOR: Envia Coordenadas de Posição (GPS Local/Mapeado)
        # -------------------------------------------------------------
        self.pub_pos = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.pub_raw = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        
        # Serviços MAVROS
        self.arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.takeoff = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.soltar_gancho = rospy.ServiceProxy('/servo_sequence_service/run', ServoSequence)

        # Subscrições
        rospy.Subscriber("mavros/state", State, self.state_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/visao/erro_alvo", Point, self.erro_cb)
        rospy.Subscriber("/visao/erro_mangueira", Point, self.erro_mangueira_cb)
        rospy.Subscriber("/visao/mangueira_orientacao", Point, self.yaw_cb)
        rospy.Subscriber("/visao/erro_base", Point, self.erro_base_cb)

        rospy.on_shutdown(self.safe_shutdown)

    def state_cb(self, msg): self.estado = msg
    def pose_cb(self, msg): self.pose_atual = msg
    def erro_cb(self, msg):
        self.erro_x = msg.x
        self.erro_y = msg.y
        self.ultima_visao_bola = rospy.Time.now()
    def erro_mangueira_cb(self, msg):
        self.erro_m_x = msg.x
        self.erro_m_y = msg.y
        self.ultima_visao_mangueira = rospy.Time.now()
    def yaw_cb(self, msg):
        self.mangueira_yaw = msg.z
    def erro_base_cb(self, msg):
        self.erro_base_x = msg.x
        self.erro_base_y = msg.y
        self.ultima_visao_base = rospy.Time.now()

    def esperar_conexao(self):
        while not rospy.is_shutdown() and not self.estado.connected:
            self.rate.sleep()

    def obter_yaw_atual(self):
        """ Extrai o ângulo de Yaw (bússola) do drone a partir da Quaternion """
        q = self.pose_atual.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_line_err(angle):
        """Envolve angulo para [-pi/2, +pi/2). Trata linha (sem direcao):
        angle e angle+pi sao equivalentes. Substitui atan(tan(x)) sem singularidade."""
        a = (angle + math.pi) % (2 * math.pi) - math.pi  # para [-pi, pi]
        if a > math.pi / 2:
            a -= math.pi
        elif a < -math.pi / 2:
            a += math.pi
        return a

    def camera_para_mapa(self, erro_frente, erro_lado, yaw_drone):
        """ Converte o erro da câmera (Frente/Lado) para o mapa X/Y do MAVROS """
        # Nota: Ajuste o sinal do 'erro_lado' dependendo se sua câmera considera Y positivo para a direita ou esquerda.
        dx = (erro_frente * math.cos(yaw_drone)) - (erro_lado * math.sin(yaw_drone))
        dy = (erro_frente * math.sin(yaw_drone)) + (erro_lado * math.cos(yaw_drone))
        return dx, dy

    def send_body_offset(self, dx, dy, dz=0.0, dyaw=None):
        """Publica setpoint relativo no body frame (FRAME_BODY_OFFSET_NED = 9).
        dx = frente (+) / tras (-),  dy = esquerda (+) / direita (-),  dz = cima (+) / baixo (-).
        dyaw = giro relativo em rad (None -> nao mexe no yaw). Positivo = anti-horario (CCW).
        FC substitui a meta a cada mensagem -> ok para chamar a 10 Hz como P-loop.
        """
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE)
        if dyaw is None:
            mask |= PositionTarget.IGNORE_YAW
        else:
            target.yaw = float(dyaw)
        target.type_mask = mask
        target.position.x = dx
        target.position.y = dy
        target.position.z = dz
        self.pub_raw.publish(target)

    def decolar(self, alt):

        self.set_mode(custom_mode="GUIDED")
        self.rate.sleep()
            
        self.arm(True)
        self.rate.sleep()
            
        self.takeoff(altitude=alt)
        rospy.loginfo(f"Decolando para {alt}m. Aguardando estabilização...")
        rospy.sleep(10.0) 

    def executar_missao_completa(self, alt_voo):
        rospy.loginfo("Iniciando busca em Zig-Zag...")
        
        ALVO_ANGULO_TELA = math.pi
        
        # Suavizador: 0.5 significa que o SetPoint é colocado na metade do caminho do erro real.
        # Isso faz o drone seguir o alvo suavemente, sem dar solavancos.
        ganho_rastreio = 0.5

        # Metros de body-offset por unidade de erro normalizado [-1, +1] da camera.
        # Usado em BOLA via send_body_offset() como ganho do P-loop visual.
        ganho_body_m = 0.4

        # Variáveis da Varredura (Posições iniciais do ZigZag)
        origem_x = self.pose_atual.pose.position.x
        origem_y = self.pose_atual.pose.position.y
        raio_zz = 2.0; theta_zz = 0.0 
        
        estado = "PROCURA"
        tempo_estabilizado = 0
        inicio_estabilizacao = rospy.Time.now()

        # Substeps de POUSO_BASE
        pouso_substep = "VOAR_ORIGEM"
        t_inicio_alinhar_base = rospy.Time.now()

        while not rospy.is_shutdown():
            t_sem_bola = (rospy.Time.now() - self.ultima_visao_bola).to_sec()
            t_sem_mangueira = (rospy.Time.now() - self.ultima_visao_mangueira).to_sec()
            t_sem_base = (rospy.Time.now() - self.ultima_visao_base).to_sec()

            # Cria a mensagem de posição alvo baseada em onde o drone já está
            alvo = PoseStamped()
            alvo.pose.position.z = alt_voo
            alvo.pose.orientation.w = 1.0  # identity quat (yaw fica gerenciado pelo FC)
            yaw_atual = self.obter_yaw_atual()

            # --- ESTADO 1: PROCURA (ZIG-ZAG POR POSIÇÃO) ---
            if estado == "PROCURA":
                if t_sem_bola < 0.5:
                    rospy.loginfo("Alvo visto! Trocando para rastreio de Posição da BOLA.")
                    estado = "BOLA"
                else:
                    # Calcula o próximo waypoint local do zig-zag
                    alvo_zz_x = origem_x + (raio_zz * math.cos(theta_zz))
                    alvo_zz_y = origem_y + (raio_zz * math.sin(theta_zz))
                    alvo.pose.position.x = alvo_zz_x
                    alvo.pose.position.y = alvo_zz_y

                    # Se chegou perto do waypoint atual, gera o próximo
                    dist_wp = math.hypot(alvo_zz_x - self.pose_atual.pose.position.x, alvo_zz_y - self.pose_atual.pose.position.y)
                    if dist_wp < 0.3:
                        theta_zz += 0.8
                        if theta_zz >= 2*math.pi: theta_zz = 0; raio_zz += 0.5

            # --- ESTADO 2: BOLA (RASTREIO FOCADO via body_offset) ---
            elif estado == "BOLA":
                if t_sem_bola < 1.0:
                    # erro_x/erro_y normalizados [-1,+1] -> metros de offset no body frame.
                    # FC substitui meta a cada msg -> 10 Hz body_offset == P-loop visual.
                    self.send_body_offset(self.erro_x * ganho_body_m,
                                          -self.erro_y * ganho_body_m,
                                          0.0)

                    dist_bola_m = math.hypot(self.erro_x, self.erro_y)
                    if dist_bola_m < 0.35:
                        rospy.loginfo("Alinhado com a bola! Iniciando procedimento MANGUEIRA.")
                        estado = "MANGUEIRA"
                else:
                    if t_sem_bola > 2.0:
                        rospy.loginfo("Alvo perdido. Retomando varredura.")
                        origem_x = self.pose_atual.pose.position.x # Recomeça o zigzag daqui
                        origem_y = self.pose_atual.pose.position.y
                        estado = "PROCURA"

            # --- ESTADO 3: MANGUEIRA (ALINHAMENTO E SOLTURA via body_offset) ---
            elif estado == "MANGUEIRA":
                if t_sem_mangueira < 1.5:
                    # Wrap [-pi/2, +pi/2] tratando linha como sem-direcao (mangueira).
                    erro_yaw = self._wrap_line_err(ALVO_ANGULO_TELA - self.mangueira_yaw)

                    # Deadband perto de ±pi/2: nessa zona o sistema eh ambiguo
                    # (perpendicular -> dois rumos validos), nao mexer no yaw evita oscilacao.
                    if abs(erro_yaw) > (math.pi / 2 - 0.2):
                        yaw_cmd = 0.0
                    else:
                        yaw_cmd = erro_yaw * 0.5

                    self.send_body_offset(0.0,
                                          self.erro_m_y * ganho_body_m,
                                          0.0,
                                          dyaw=yaw_cmd)

                    dist_lateral = abs(self.erro_m_y)
                    
                    if dist_lateral < 0.10 and abs(erro_yaw) < 0.15:
                        if tempo_estabilizado == 0:
                            inicio_estabilizacao = rospy.Time.now()
                            tempo_estabilizado = 1
                        elif (rospy.Time.now() - inicio_estabilizacao).to_sec() > 1.0:
                            rospy.loginfo("MISSÃO CONCLUÍDA. SOLTANDO GANCHO!")
                            try:
                                resp = self.soltar_gancho(servo=5, pwm_first=1690, pwm_second=1100, delay_sec=3.0)
                                if not resp.success:
                                    rospy.logwarn("Falha ao soltar gancho: %s", resp.message)
                            except rospy.ServiceException as e:
                                rospy.logerr("Servico servo_sequence indisponivel: %s", e)
                            rospy.loginfo("Iniciando retorno e pouso na base azul.")
                            estado = "POUSO_BASE"
                            pouso_substep = "VOAR_ORIGEM"
                    else:
                        tempo_estabilizado = 0
                else:
                    if t_sem_mangueira > 3.0:
                        rospy.loginfo("Mangueira perdida. Recalibrando pela bola.")
                        estado = "BOLA"

            # --- ESTADO 4: POUSO_BASE (volta para origem, alinha visualmente, pousa) ---
            elif estado == "POUSO_BASE":
                if pouso_substep == "VOAR_ORIGEM":
                    alvo.pose.position.x = 0.0
                    alvo.pose.position.y = 0.0
                    dist_origem = math.hypot(self.pose_atual.pose.position.x,
                                             self.pose_atual.pose.position.y)
                    if dist_origem < 0.5:
                        rospy.loginfo("Chegou na origem. Procurando base azul...")
                        pouso_substep = "ALINHAR_BASE"
                        t_inicio_alinhar_base = rospy.Time.now()

                elif pouso_substep == "ALINHAR_BASE":
                    if t_sem_base < 1.0:
                        # Body offset visual como em BOLA
                        self.send_body_offset(self.erro_base_x * ganho_body_m,
                                              self.erro_base_y * ganho_body_m,
                                              0.0)
                        dist_base = math.hypot(self.erro_base_x, self.erro_base_y)
                        if dist_base < 0.15:
                            rospy.loginfo("Centralizado na base. Pousando.")
                            self.set_mode(custom_mode="LAND")
                            return True
                    else:
                        # Base nao vista: mantem hover na origem por timeout, depois LAND no local
                        alvo.pose.position.x = 0.0
                        alvo.pose.position.y = 0.0
                        if (rospy.Time.now() - t_inicio_alinhar_base).to_sec() > 8.0:
                            rospy.logwarn("Base nao encontrada. LAND na origem.")
                            self.set_mode(custom_mode="LAND")
                            return True

            # Publica o SETPOINT DE POSIÇÃO. Estados que usam body_offset (BOLA,
            # MANGUEIRA, POUSO_BASE/ALINHAR_BASE com base visivel) pulam essa publicacao.
            # Em MANGUEIRA sem visao: nada eh publicado -> FC segura o ultimo alvo (body_offset latch).
            usando_body_offset = (
                estado == "BOLA" or
                estado == "MANGUEIRA" or
                (estado == "POUSO_BASE" and pouso_substep == "ALINHAR_BASE" and t_sem_base < 1.0)
            )
            if not usando_body_offset:
                self.pub_pos.publish(alvo)
            self.rate.sleep()

    def retornar_e_pousar(self):
        rospy.loginfo("Iniciando procedimento de pouso...")
        self.set_mode(custom_mode="LAND")

    def safe_shutdown(self):
        """Retorna para a origem (0,0,alt) e pousa. Chamado via rospy.on_shutdown."""
        rospy.loginfo("Shutdown iniciado. Retornando para a origem...")
        try:
            alvo = PoseStamped()
            alvo.pose.position.x = 0.0
            alvo.pose.position.y = 0.0
            alvo.pose.position.z = ALTURA_MISSAO
            alvo.pose.orientation.w = 1.0
            self.pub_pos.publish(alvo)

            # Espera dentro do raio de 1m da origem (tempo limite ~15s para nao travar shutdown)
            t_limite = rospy.Time.now() + rospy.Duration(15.0)
            while rospy.Time.now() < t_limite and (
                abs(self.pose_atual.pose.position.x) > 1.0 or
                abs(self.pose_atual.pose.position.y) > 1.0
            ):
                self.pub_pos.publish(alvo)
                rospy.sleep(0.5)

            rospy.sleep(2.0)
            self.set_mode(custom_mode="LAND")
            self.arm(False)
            rospy.loginfo("LAND e desarme enviados com sucesso")
        except rospy.ServiceException:
            rospy.logwarn("Servicos MAVROS indisponiveis durante shutdown")

        rospy.loginfo("Saida forcada para encerrar o no")
        os._exit(0)

if __name__ == "__main__":
    try:
        drone = Missao2EletroQuad()
        drone.esperar_conexao()
        
        drone.decolar(ALTURA_MISSAO)
        
        if drone.executar_missao_completa(ALTURA_MISSAO):
            drone.retornar_e_pousar() 
                
    except rospy.ROSInterruptException:
        pass
