# Arena DroneIFSC - ROS2 Humble & ArduPilot SITL

> **Destinado à Competição Brasileira de Robótica (CBR) 2026**

Repositório oficial para a simulação de veículos aéreos não tripulados (UAVs) em ambiente de arena em espaço confinado, desenvolvido para a **Competição Brasileira de Robótica (CBR) 2026**, utilizando a arquitetura moderna do **ROS2 Humble**, **Gazebo 11** e o piloto automático **ArduPilot SITL**.

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
* Ter criado uma pasta de workspace (ex: `~/ros2_ws/src`) no seu ambiente host.

---

## 1. Criação e Configuração do Ambiente (Docker)

1. **Clone este repositório na sua workspace:**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/herickbentoeletronica/droneifsc_gazebo_ros2.git
```

2. **Crie e acesse o container Docker:**
```bash
docker run -it -d --name arena_ros2   --net=host   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix   -v ~/.Xauthority:/root/.Xauthority:rw   -v ~/ros2_ws:/root/ros2_ws   osrf/ros:humble-desktop

docker exec -it arena_ros2 bash
```

---

## 2. Instalação de Dependências, ArduPilot e MAVProxy (Dentro do Container)

Com o container aberto, instale as ferramentas de voo e a ponte MAVROS:

```bash
# Atualizar e instalar ferramentas base
apt update && apt install git cmake build-essential python3-pip python3-dev libgazebo11-dev -y

# Instalar MAVProxy e utilitários Python
pip3 install PyYAML mavproxy pymavlink pexpect future --user
export PATH=$PATH:$HOME/.local/bin

# Instalar MAVROS para ROS2 Humble
apt install ros-humble-mavros ros-humble-mavros-extras -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

# Clonar e compilar o ArduPilot SITL
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter
```

---

## 3. Como Executar o Projeto

Para rodar a simulação completa com sucesso, você precisará abrir **três terminais independentes dentro do container** (`docker exec -it arena_ros2 bash`):

### Terminal 1: Iniciar a Arena (Gazebo + ROS2)
```bash
cd /root/ros2_ws
colcon build --packages-select droneifsc
source install/setup.bash
ros2 launch droneifsc espacoconfinado.launch.py
```

### Terminal 2: Iniciar o Cérebro do Drone (SITL / MAVProxy)
Inicie o simulador do ArduPilot.

*Nota técnica:* Ao contrário do ROS 1, no ecossistema ROS2 a inicialização padrão gerencia a comunicação diretamente pelas portas padrão do SITL, dispensando parâmetros manuais adicionais de saída na inicialização:
```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

### Terminal 3: Conectar a Ponte ROS2 (MAVROS)
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
*Desenvolvido pela Equipe DroneIFSC para a CBR 2026.*
