cmake_minimum_required(VERSION 3.8)
project(template_ros_2_package)

# Define a política do CMake para encontrar o Python, corrigindo os avisos de compilação.
cmake_policy(SET CMP0148 OLD)

# --- 1. Padrão C++ e Opções de Compilação ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 2. Encontrar Dependências ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED) # Necessário para registrar componentes
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- 3. Gerar Mensagens e Serviços ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MensagemExemplo.msg"
  "srv/ServicoExemplo.srv"
)

# --- 4. Definir Bibliotecas, Executáveis e Dependências ---
include_directories(include)

# Cria uma "biblioteca virtual" com as interfaces geradas.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -- SimpleNode --
# A) Compila a classe SimpleNode como uma biblioteca.
add_library(simple_node_lib SHARED
  src/simple_node.cpp
)
ament_target_dependencies(simple_node_lib rclcpp rclcpp_components std_msgs)
target_link_libraries(simple_node_lib ${cpp_typesupport_target})

# B) Cria um executável simples que usa a biblioteca.
add_executable(simple_node
  src/simple_node_main.cpp
)
target_link_libraries(simple_node simple_node_lib)

# C) Registra o SimpleNode como um componente ROS 2.
rclcpp_components_register_nodes(simple_node_lib
  PLUGIN "template_ros_2_package::SimpleNode"
)

# -- LifecycleNode --
# A) Compila a classe LifecycleNode como uma biblioteca.
add_library(lifecycle_node_lib SHARED
  src/lifecycle_node.cpp
)
ament_target_dependencies(lifecycle_node_lib rclcpp_lifecycle rclcpp_components std_msgs)
target_link_libraries(lifecycle_node_lib ${cpp_typesupport_target})

# B) Cria um executável simples que usa a biblioteca.
add_executable(lifecycle_node
  src/lifecycle_node_main.cpp
)
target_link_libraries(lifecycle_node lifecycle_node_lib)

# C) Registra o LifecycleNode como um componente ROS 2.
rclcpp_components_register_nodes(lifecycle_node_lib
  PLUGIN "template_ros_2_package::LifecycleNode"
)

# --- 5. Instalação ---
# Instala as bibliotecas e os executáveis.
install(TARGETS
  simple_node_lib
  simple_node
  lifecycle_node_lib
  lifecycle_node
  DESTINATION lib
)

# Instala os ficheiros de cabeçalho, launch, params, msg e srv.
install(DIRECTORY
  include/
  DESTINATION include
)
install(DIRECTORY
  launch
  params
  msg
  srv
  DESTINATION share/${PROJECT_NAME}/
)

# --- 6. Exportar Dependências e Registrar Pacote ---
ament_export_dependencies(rosidl_default_runtime rclcpp rclcpp_lifecycle rclcpp_components std_msgs)

ament_package()