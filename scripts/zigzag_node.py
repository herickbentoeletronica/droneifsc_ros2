#!/usr/bin/env python3
import math
import os

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, TransformStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
import tf2_ros

ALT = 1.2


class MissionFrame:
    """Mantem um frame fixo no ponto de inicio da missao."""

    def __init__(self):
        self.local_frame = rospy.get_param("~local_frame", "map")
        self.frame_id = rospy.get_param("~mission_frame", "mission_start")
        self.ready = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def capture(self, local_position, yaw_enu):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.local_frame
        transform.child_frame_id = self.frame_id
        transform.transform.translation.x = local_position.x
        transform.transform.translation.y = local_position.y
        transform.transform.translation.z = local_position.z

        q = quaternion_from_euler(0.0, 0.0, yaw_enu)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.static_tf_broadcaster.sendTransform(transform)
        if hasattr(self.tf_buffer, "set_transform_static"):
            self.tf_buffer.set_transform_static(transform, "mission_node")

        self.ready = True
        rospy.loginfo(
            f"mission_start capturado em {self.local_frame}: "
            f"x={local_position.x:.2f}, y={local_position.y:.2f}, "
            f"z={local_position.z:.2f}, yaw={math.degrees(yaw_enu):.1f}deg"
        )

    def to_local_pose(self, x_forward, y_right, z_up):
        mission_pose = PoseStamped()
        mission_pose.header.stamp = rospy.Time(0)
        mission_pose.header.frame_id = self.frame_id
        mission_pose.pose.position.x = x_forward
        mission_pose.pose.position.y = -y_right  # API: direita positiva; TF: esquerda positiva.
        mission_pose.pose.position.z = z_up
        mission_pose.pose.orientation.w = 1.0

        transform = self.tf_buffer.lookup_transform(
            self.local_frame, self.frame_id, rospy.Time(0), rospy.Duration(0.2)
        )
        return tf2_geometry_msgs.do_transform_pose(mission_pose, transform)

    def to_mission_position(self, local_position):
        local_pose = PoseStamped()
        local_pose.header.stamp = rospy.Time(0)
        local_pose.header.frame_id = self.local_frame
        local_pose.pose.position = local_position
        local_pose.pose.orientation.w = 1.0

        transform = self.tf_buffer.lookup_transform(
            self.frame_id, self.local_frame, rospy.Time(0), rospy.Duration(0.1)
        )
        mission_pose = tf2_geometry_msgs.do_transform_pose(local_pose, transform)
        return Point(
            mission_pose.pose.position.x,
            -mission_pose.pose.position.y,
            mission_pose.pose.position.z
        )


class Missao1Eletroquad:
    def __init__(self):
        rospy.init_node('zigzag_vel_node', anonymous=False)

        self.estado = State()
        self.pos = PoseStamped()
        self.current_pose = None
        self.current_yaw_enu = 0.0

        self.mission_frame = MissionFrame()

        # Variaveis de Pouso
        self.pouso_acionado = False
        self.alvo_x = 0.0
        self.alvo_y = 0.0

        # Variaveis de Correcao Visual (Visual Servoing)
        self.erro_x = 0.0
        self.erro_y = 0.0
        self.ultima_visao_tempo = rospy.Time(0)

        # Subscribers
        rospy.Subscriber("mavros/state", State, self.state_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pos_cb)
        rospy.Subscriber("/visao/alvo_pouso", Point, self.alvo_cb)
        rospy.Subscriber("/visao/erro_alvo", Point, self.erro_visao_cb)

        # Publishers
        self.pub_vel = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.pub_pos = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpoint_raw_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        # Serviços
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/cmd/takeoff')
        rospy.wait_for_service('mavros/cmd/land')

        self.arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.takeoff = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.land = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
        self.rate = rospy.Rate(10)

        # Handler de shutdown para pouso seguro
        rospy.on_shutdown(self.safe_shutdown)
        
    def state_cb(self, msg):
        self.estado = msg

    def pos_cb(self, msg):
        self.pos = msg

        q = msg.pose.orientation
        _, _, self.current_yaw_enu = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.mission_frame.ready:
            try:
                self.current_pose = self.mission_frame.to_mission_position(msg.pose.position)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.current_pose = Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        else:
            self.current_pose = Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def alvo_cb(self, msg):
        if not self.pouso_acionado:
            self.alvo_x = msg.x
            self.alvo_y = msg.y
            self.pouso_acionado = True
            rospy.logwarn(f"ALERTA GERAL: Alvo recebido (X:{self.alvo_x:.2f}, Y:{self.alvo_y:.2f}). Abortando busca!")

    def erro_visao_cb(self, msg):
        self.erro_x = msg.x
        self.erro_y = msg.y
        self.ultima_visao_tempo = rospy.Time.now()

    def esperar_conexao(self):
        rospy.loginfo("Aguardando conexao com a Pixhawk...")
        while not rospy.is_shutdown() and not self.estado.connected:
            self.rate.sleep()
        rospy.loginfo("Conectado!")

    def capture_mission_start_frame(self):
        """Fixa o frame mission_start na pose/yaw atuais antes da decolagem."""
        try:
            msg = rospy.wait_for_message("mavros/local_position/pose", PoseStamped, timeout=5.0)
            self.pos_cb(msg)
        except rospy.ROSException:
            rospy.logwarn("Timeout esperando pose local; mission_start nao capturado")
            return False
        self.mission_frame.capture(self.pos.pose.position, self.current_yaw_enu)
        self.current_pose = self.mission_frame.to_mission_position(self.pos.pose.position)
        return True

    def send_position(self, x, y, z):
        """Publica setpoint absoluto no frame mission_start. x=frente, y=direita, z=cima."""
        if not self.mission_frame.ready:
            rospy.logwarn_throttle(2.0, "mission_start ainda nao capturado; usando frame local MAVROS")
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation = self.pos.pose.orientation
            self.pub_pos.publish(msg)
            return

        try:
            local_msg = self.mission_frame.to_local_pose(x, y, z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn(f"Falha ao transformar setpoint mission_start->{self.mission_frame.local_frame}: {exc}")
            return

        local_msg.header.stamp = rospy.Time.now()
        local_msg.header.frame_id = self.mission_frame.local_frame
        self.pub_pos.publish(local_msg)

    def armar_e_decolar(self, alt):
        self.capture_mission_start_frame()

        vel = TwistStamped()
        for _ in range(50):
            self.pub_vel.publish(vel)
            self.rate.sleep()

        self.set_mode(custom_mode="GUIDED")

        self.arm(True)

        rospy.loginfo(f"Decolando para {alt} metros...")
        self.takeoff(altitude=alt)

        timeout = rospy.Time.now().to_sec() + 10
        while self.pos.pose.position.z < alt * 0.60:
            if timeout < rospy.Time.now().to_sec():
                rospy.loginfo(f"Timeout de chegar na altura")
                break
            self.rate.sleep()

        rospy.loginfo("Altitude atingida")


    def executar_zigzag(self, tam_x, tam_y, passo, alt):
        x_min, x_max = -tam_x/2.0, tam_x/2.0
        y_min, y_max = -tam_y/2.0, tam_y/2.0

        y = y_min
        direcao = 1

        self.mover_para(x_min, y_min, alt)

        while y <= y_max and not rospy.is_shutdown():
            if getattr(self, 'pouso_acionado', False):
                return

            x_dest = x_max if direcao == 1 else x_min
            self.mover_para(x_dest, y, alt)

            y += passo
            direcao *= -1

            if y <= y_max:
                self.mover_para(x_dest, y, alt)

    def mover_para(self, x_alvo, y_alvo, z_alvo, tol=0.3, ignorar_interrupcao=False, timeout=30.0):
        tempo_inicio = rospy.Time.now()

        while not rospy.is_shutdown():
            if getattr(self, 'pouso_acionado', False) and not ignorar_interrupcao:
                return

            if (rospy.Time.now() - tempo_inicio).to_sec() > timeout:
                break

            if self.current_pose is None:
                self.rate.sleep()
                continue

            dist = math.sqrt(
                (x_alvo - self.current_pose.x) ** 2 +
                (y_alvo - self.current_pose.y) ** 2 +
                (z_alvo - self.current_pose.z) ** 2
            )

            if dist < tol:
                break

            self.send_position(x_alvo, y_alvo, z_alvo)
            self.rate.sleep()

        rospy.sleep(1.0)


    def send_body_offset(self, dx, dy, dz=0.0):
        """Setpoint relativo no body frame (FRAME_BODY_OFFSET_NED = 9).
        dx = frente (+) / tras (-)
        dy = direita (+) / esquerda (-)
        dz = cima (-) / baixo (+)   <- NED, MAS tem algum erro de conversao ai
        """
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        target.position.x = dx
        target.position.y = dy
        target.position.z = dz
        self.setpoint_raw_pub.publish(target)

    def centralizar_e_pousar(self):
        """Centraliza no alvo via send_body_offset + desce em N passos. Pousa no fim.

        erro_x e erro_y vem de visionnode em pixel NORMALIZADO [-1..1].
        Convertemos para metros body via GAIN. Sem velocity, so PositionTarget BODY_OFFSET.
        """
        rospy.loginfo("================================================")
        rospy.loginfo(" Centralizando + pousando (body offset) ")
        rospy.loginfo("================================================")

        MAX_DESCIDAS = 5
        DESCEND_STEP = 0.3         # m por descida (NED, positivo = pra baixo)
        GAIN = 1                 # delta_norm * GAIN = metros body
        ERR_THRESHOLD = 0.04       # delta normalizada -> centralizado
        MAX_STEP = 0.8             # m por iteracao XY
        LOST_TIMEOUT = 7        # s sem ver
        SETTLE_TIME = 4          # s entre comandos

        final_err = float('inf')

        for descida in range(MAX_DESCIDAS):
            rospy.loginfo(f"Descida {descida+1}/{MAX_DESCIDAS}")

            # 1. CENTRALIZAR (se temos visao recente)
            tempo_sem_ver = (rospy.Time.now() - self.ultima_visao_tempo).to_sec()
            if tempo_sem_ver < LOST_TIMEOUT:
                ex_norm, ey_norm = self.erro_x, self.erro_y
                final_err = math.sqrt(ex_norm**2 + ey_norm**2)

                if abs(ex_norm) > ERR_THRESHOLD or abs(ey_norm) > ERR_THRESHOLD:
                    # Mapeamento delta -> body (testar empiricamente os sinais)
                    dx = GAIN * ex_norm
                    dy = -GAIN * ey_norm
                    if dx >  MAX_STEP: dx =  MAX_STEP
                    if dx < -MAX_STEP: dx = -MAX_STEP
                    if dy >  MAX_STEP: dy =  MAX_STEP
                    if dy < -MAX_STEP: dy = -MAX_STEP
                    rospy.loginfo(f"  centralize: delta=({ex_norm:.2f},{ey_norm:.2f}) step=({dx:.2f},{dy:.2f})")
                    self.send_body_offset(dx, dy, 0.0)
                    rospy.sleep(SETTLE_TIME)
                    #continue   # tenta centralizar de novo antes de descer
            else:
                rospy.logwarn(f"  alvo perdido ({tempo_sem_ver:.1f}s), desce sem centralizar")

            # 2. DESCER um passo (mantem XY)
            rospy.loginfo(f"  descendo {DESCEND_STEP}m (NED, body)")
            self.send_body_offset(0.0, 0.0, -DESCEND_STEP)
            rospy.sleep(SETTLE_TIME)

        # 3. Decisao final
        if final_err < ERR_THRESHOLD:
            rospy.loginfo(f"Centralizado (err={final_err:.2f}), acionando LAND")
        else:
            rospy.logwarn(f"NAO centralizado (err={final_err:.2f}) apos {MAX_DESCIDAS} descidas, LAND mesmo assim")

        self.set_mode(custom_mode="LAND")

    def safe_shutdown(self):
        rospy.loginfo("Shutdown iniciado. Tentando pousar...")
        try:
            rospy.loginfo("Retornando para a origem")
            self.send_position(0, 0, ALT)

            t_start = rospy.Time.now()
            while self.current_pose is not None and (
                abs(self.current_pose.x) > 1 or abs(self.current_pose.y) > 1
            ):
                if (rospy.Time.now() - t_start).to_sec() > 15:
                    break
                self.send_position(0, 0, ALT)
                rospy.sleep(0.5)
            rospy.sleep(2)

            self.set_mode(custom_mode="LAND")
            self.arm(False)
            rospy.loginfo("Pouso e desarme enviados com sucesso")

        except rospy.ServiceException:
            rospy.logwarn("Servicos MAVROS indisponiveis durante shutdown")

        rospy.loginfo("Saida forcada para encerrar o no")
        os._exit(0)

if __name__ == "__main__":
    try:
        drone = Missao1Eletroquad()
        drone.esperar_conexao()

        drone.armar_e_decolar(ALT)
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if not drone.pouso_acionado:
                drone.executar_zigzag(8, 4, 1, ALT)
            else:
                rospy.loginfo(f"Navegando para as imediacoes da base (X:{drone.alvo_x:.2f}, Y:{drone.alvo_y:.2f})")
                # 1. Volta rapidamente para a coordenada global onde a base foi avistada
                drone.mover_para(drone.alvo_x, drone.alvo_y, ALT, ignorar_interrupcao=True)

                # 2. Transfere a autoridade de voo para o rastreio ativo da câmera
                drone.centralizar_e_pousar()

                rospy.loginfo("Missao Concluida! Desligando no de navegacao.")
                break

    except rospy.ROSInterruptException:
        pass
