# 📦 Pacote Template ROS 2 - Estrutura Didática

Este pacote ROS 2 foi desenvolvido com fins didáticos, servindo como ponto de partida para novos projetos. Ele contém uma estrutura completa, modular e comentada, facilitando o aprendizado dos principais conceitos do ROS 2.

## 🧠 Conteúdo do Pacote

### 🧩 Estrutura de Comunicação ROS 2

- Publicador (Publisher): Envia mensagens para um tópico em intervalos regulares (timer).
- Assinante (Subscriber): Recebe e processa mensagens de um tópico.
- Serviço (Service Server): Responde a requisições de outros nós.
- Cliente (Service Client): Realiza chamadas de serviço.
- Temporizador (Timer): Dispara eventos em intervalos periódicos.

### ⚙️ Parâmetros ROS 2

- Leitura de parâmetros através de arquivos .yaml.
- Uso de parâmetros para controlar comportamento do nó.

### 🧾 Tipos Customizados

- Mensagem (.msg) personalizada: Para comunicação específica.
- Serviço (.srv) personalizado: Para chamadas de serviço com estrutura própria.

### 🚀 Launch File

- Arquivo .launch.py que executa todos os nós com seus parâmetros configurados.
- Permite execução modular e parametrizada.

## 🛠️ Como Usar

### 🔧 Compilação do Pacote

Clone este repositório dentro do diretório src do seu workspace ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/wagnerdgarcia/template_ros_2_package.git
```

Volte para a raiz do workspace e compile o pacote:

```bash
cd ~/ros2_ws
colcon build --packages-select template_ros_2_package
```

Após a compilação, adicione o pacote ao ambiente do ROS 2:

```bash
source install/setup.bash
```

### ▶️ Execução dos Nós

Existem duas maneiras principais de executar os nós deste pacote.

#### Método 1: Executar um Único Nó (Ideal para Testes)

Você pode executar um nó individualmente usando o comando ros2 run. Isso é útil para testar um nó de forma isolada.

Para executar apenas o SimpleNode:

```bash
ros2 run template_ros_2_package simple_node
```

Observação: Ao usar ros2 run desta forma, o nó será iniciado com os parâmetros padrão definidos no código, e não com os valores do arquivo simple_node.yaml.

#### Método 2: Executar o Sistema Completo com Parâmetros (Recomendado)

A forma correta e recomendada de iniciar o sistema é usando o arquivo de inicialização (launch file). Este método garante que todos os nós (SimpleNode e LifecycleNode) sejam iniciados e que seus respectivos parâmetros dos arquivos .yaml sejam carregados.

```bash
ros2 launch template_ros_2_package template.launch.py
```

Após executar este comando, você poderá interagir com os nós usando as ferramentas do ROS 2:

- Para ver os tópicos: `ros2 topic list`
- Para ver os serviços: `ros2 service list`
- Para interagir com o ciclo de vida: `ros2 lifecycle get /meu_lifecycle_node`
