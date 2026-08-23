import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # Encontra a pasta do seu pacote compilado
    pkg_dir = get_package_share_directory('droneifsc')
    
    # Caminhos para o mundo e para a pasta de modelos
    world_file = os.path.join(pkg_dir, 'worlds', 'espacoconfinado.world')
    models_dir = os.path.join(pkg_dir, 'models')

    # Força o Gazebo a olhar para a pasta models do seu pacote
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{models_dir}:{gazebo_model_path}" if gazebo_model_path else models_dir

    return LaunchDescription([
        # Seta a variavel de ambiente automaticamente antes de abrir o Gazebo
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=new_model_path),
        
        # Inicia o Gazebo com as bibliotecas corretas do ROS2
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
            output='screen'
        )
    ])
