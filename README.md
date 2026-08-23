# Arena Drone IFSC - ROS2 Humble & ArduPilot SITL

Repositório oficial para a simulação de veículos aéreos não tripulados (UAVs) em ambiente de arena em espaço confinado, utilizando a arquitetura moderna do ROS2 Humble, Gazebo 11 e o piloto automático ArduPilot SITL.

---

## Arquitetura do Projeto
Este ecossistema foi migrado para uma estrutura conteinerizada via Docker, garantindo portabilidade total e isolamento de dependências. O projeto divide-se em:
* **Física e Mundo (Gazebo):** Simulação tridimensional do labirinto confinado e do drone com sensores de câmera.
* **Cérebro de Voo (ArduPilot SITL & MAVProxy):** Controladora de voo virtual que processa a dinâmica do veículo e aceita comandos de navegação.
* **Ponte de Comunicação (MAVROS):** Tradutor oficial que conecta o ecossistema ArduPilot ao ecossistema ROS2.

---

## Pré-requisitos
* Sistema operacional compatível com Docker e WSL2.
* Docker Desktop configurado com suporte a aplicações gráficas.
* ROS2 Humble Desktop & Gazebo 11 instalados no container.

---

## Estrutura de Pastas
* `launch/`: Scripts de inicialização em Python (`.launch.py`).
* `worlds/`: Cenários e mapas do Gazebo (`.world`).
* `models/`: Modelos 3D autossuficientes do labirinto, pisos e do drone (`droneIFSC`).

---

## Como Executar o Projeto

Para rodar a simulação completa com sucesso, você precisará abrir **três terminais independentes**, acessando o container em cada um deles (`docker exec -it arena_ros2_gpu bash`):

### 1. Terminal 1: Iniciar a Arena (Gazebo + ROS2)
```bash
cd ~/ros2_ws
colcon build --packages-select droneifsc
source install/setup.bash
ros2 launch droneifsc espacoconfinado.launch.py
```

### 2. Terminal 2: Iniciar o Cérebro do Drone (SITL / MAVProxy)
```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
*(Após o carregamento, digite no prompt do MAVProxy para decolar: `mode guided`, depois `arm throttle`, e por fim `takeoff 2`)*.

### 3. Terminal 3: Conectar a Ponte ROS2 (MAVROS)
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@
```

---

## Validação e Tópicos
Para inspecionar os dados da simulação em tempo de execução:
* **Listar nós ativos:** `ros2 node list`
* **Verificar tópicos de telemetria e câmera:** `ros2 topic list`
* **Visualizar imagem da câmera em tempo real:** Utilize o `rviz2` ajustando o *Fixed Frame* para `cam_link`.

---
*Desenvolvido pelo IFSC para pesquisa e desenvolvimento em robótica autônoma e sistemas embarcados.*
