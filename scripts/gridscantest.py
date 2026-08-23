import time
import math
import numpy as np
from pymavlink import mavutil

def vehicle_connection(option="sitl", mavlink_id=1):
    if option == "serial":
        connect_string = '/dev/ttyAMA0'
        baud_rate = 57600
    elif option == "sitl":
        # Conexão padrão para ArduPilot SITL / Gazebo
        connect_string = 'udp:127.0.0.1:14550' 
        baud_rate = 115200
    else:
        print("\nConexão inválida! Usando SITL por padrão.")
        connect_string = 'udp:127.0.0.1:14550'
        baud_rate = 115200

    # Inicia a conexão
    print("Conectando em: ", connect_string)
    vehicle = mavutil.mavlink_connection(connect_string, baud=baud_rate, 
                                         source_system=mavlink_id, source_component=2)
    
    # Aguarda o primeiro heartbeat para garantir a comunicação
    vehicle.wait_heartbeat()
    print(f"Heartbeat do sistema (system {vehicle.target_system} component {vehicle.target_component})")    
    return vehicle

def arm_and_takeoff(vehicle, target_altitude):
    # Mode = GUIDED (Modo 4 no ArduCopter é o GUIDED)
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4)
    time.sleep(1)
    
    # Armar drone
    print("Armando os motores...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)

    # Esperar os motores armarem
    vehicle.motors_armed_wait()
    print("Drone armado!")

    # Enviar comando de decolagem
    print(f"Decolando para {target_altitude} metros...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, target_altitude)

    # Aguardar até atingir a altitude alvo (lendo LOCAL_POSITION_NED)
    # Lembre-se: No sistema NED (North, East, Down), o Z (Down) é negativo para altitudes acima do solo
    while True:
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            alt = -msg.z
            print(f"Altitude atual: {alt:.2f} m")
            if alt >= target_altitude * 0.95:
                print("Altitude alvo atingida!")
                break
        time.sleep(0.2)


def goto_position_local_ned(vehicle, x, y, z):
    """
    Move o drone para coordenadas locais (Norte, Leste, Para Cima).
    O parâmetro z (altitude) deve ser positivo; a função converte para o padrão NED (-z).
    """
    vehicle.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (não usado)
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000, # Máscara: usar apenas posição
        x, y, -z,  # Coordenadas X, Y e Z
        0, 0, 0,   # Velocidade (não usado)
        0, 0, 0,   # Aceleração (não usado)
        0, 0)      # Yaw (não usado)


def wait_for_position_local_ned(vehicle, target_x, target_y, target_z, tolerance=0.5, timeout=30):
    """
    Aguarda ativamente lendo mensagens até que o drone alcance a posição desejada.
    """
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            curr_x = msg.x
            curr_y = msg.y
            curr_z = -msg.z
            
            # Verifica distância
            dx = abs(target_x - curr_x)
            dy = abs(target_y - curr_y)
            dz = abs(target_z - curr_z)
            
            if dx <= tolerance and dy <= tolerance and dz <= tolerance:
                print(f" -> Chegou no alvo ({target_x}, {target_y}, {target_z})!")
                return True
                
        time.sleep(0.1)
        
    print("TIMEOUT: Falha ao alcançar a posição a tempo.")
    return False


def main():
    # 1. Conectar ao veículo
    vehicle = vehicle_connection(option="sitl")

    # 2. Definir a altitude base de navegação e decolar
    alt_base = 5.0
    arm_and_takeoff(vehicle, alt_base)
    time.sleep(2)

    # Lista de pontos (x, y) - X = Frente/Norte, Y = Direita/Leste (em metros)
    pontos = [
        (5, 5),   # Ponto 1
        (10, 0),  # Ponto 2
        (0, -5)   # Ponto 3 (Quantos pontos quiser...)
    ]

    # 3. Navegar pelos pontos realizando a lógica de Sobe/Desce
    for i, (p_x, p_y) in enumerate(pontos):
        print(f"\n--- Navegando para o Ponto {i+1} ---")
        
        # Vai para Pn (x, y) na altitude base
        print(f"Indo para ({p_x}, {p_y}), Altitude: {alt_base}m")
        goto_position_local_ned(vehicle, p_x, p_y, alt_base)
        wait_for_position_local_ned(vehicle, p_x, p_y, alt_base)
        time.sleep(1)

        # Sobe 2 metros
        print(f"Subindo 2 metros...")
        goto_position_local_ned(vehicle, p_x, p_y, alt_base + 2.0)
        wait_for_position_local_ned(vehicle, p_x, p_y, alt_base + 2.0)
        time.sleep(1)

        # Desce 2 metros
        print(f"Descendo 2 metros...")
        goto_position_local_ned(vehicle, p_x, p_y, alt_base)
        wait_for_position_local_ned(vehicle, p_x, p_y, alt_base)
        time.sleep(1)

    # 4. Voltar para casa (RTL - Return to Launch)
    print("\nFim da missão. Voltando para casa (RTL)...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
        0, 0, 0, 0, 0, 0, 0)
    
    # Aguarda o RTL finalizar (pode acompanhar pelo console do Gazebo/SITL)
    print("Comando RTL enviado com sucesso.")

if __name__ == "__main__":
    main()
