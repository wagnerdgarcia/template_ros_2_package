from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Diretório onde os arquivos de configuração estão localizados
    config_dir = os.path.join(
        get_package_share_directory('template_ros_2_package'),
        'params'
    )

    # Caminho para o arquivo de parâmetros do nó simples
    simple_node_params = os.path.join(config_dir, 'simple_node.yaml')
    
    # Caminho para o arquivo de parâmetros do nó de ciclo de vida
    lifecycle_node_params = os.path.join(config_dir, 'lifecycle_node.yaml')

    return LaunchDescription([
        # Inicia o nó simples, carregando seus parâmetros específicos
        Node(
            package='template_ros_2_package',
            executable='simple_node',
            name='meu_simple_node',
            output='screen',
            parameters=[simple_node_params]
        ),
        
        # Inicia o nó de ciclo de vida, carregando seus parâmetros específicos
        Node(
            package='template_ros_2_package',
            executable='lifecycle_node',
            name='meu_lifecycle_node',
            output='screen',
            parameters=[lifecycle_node_params]
        )
    ])