cmake_minimum_required(VERSION 3.8)
project(template_ros_2_package)

# Define o padrão C++ a ser utilizado.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# --- 1. Encontrar Dependências ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- 2. Gerar Mensagens e Serviços ---
# Adiciona uma dependência para garantir que as mensagens sejam geradas antes de serem usadas.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MensagemExemplo.msg"
  "srv/ServicoExemplo.srv"
  DEPENDENCIES std_msgs
)

# --- 3. Adicionar Diretórios de Inclusão ---
include_directories(include)

# --- 4. Definir Executáveis e Suas Dependências ---
# Cria uma biblioteca com as mensagens/serviços gerados.
set(ros_interfaces_dependencies
  rosidl_typesupport_cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  ${ros_interfaces_dependencies}
)

# Adiciona o executável para o nó simples.
add_executable(simple_node src/simple_node_main.cpp src/simple_node.cpp)
ament_target_dependencies(simple_node rclcpp std_msgs)
# Linka o nó com as mensagens/serviços.
target_link_libraries(simple_node ${cpp_typesupport_target})

# Adiciona o executável para o nó de ciclo de vida.
add_executable(lifecycle_node src/lifecycle_node_main.cpp src/lifecycle_node.cpp)
ament_target_dependencies(lifecycle_node rclcpp rclcpp_lifecycle std_msgs)
# Linka o nó com as mensagens/serviços.
target_link_libraries(lifecycle_node ${cpp_typesupport_target})

# --- 5. Instalação ---
install(TARGETS
  simple_node
  lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  msg
  srv
  DESTINATION share/${PROJECT_NAME}/
)

# --- 6. Exportar Dependências e Registrar Pacote ---
ament_export_dependencies(rosidl_default_runtime rclcpp rclcpp_lifecycle std_msgs)

ament_package()