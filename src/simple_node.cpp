#include "template_ros_2_package/simple_node.hpp"

namespace template_ros_2_package
{

    /* SimpleNode() //{ */
    SimpleNode::SimpleNode(const rclcpp::NodeOptions &options)
        : Node("meu_simple_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "Inicializando SimpleNode...");

        // Declaração e Leitura de Parâmetros
        this->declare_parameter<std::string>("meu_parametro_string", "default_string");
        this->declare_parameter<double>("meu_parametro_double", 1.0);

        this->get_parameter("meu_parametro_string", meu_parametro_string_);
        this->get_parameter("meu_parametro_double", meu_parametro_double_);

        // Inicialização de Publisher, Subscriber, Services e Timer
        publisher_ = this->create_publisher<template_ros_2_package::msg::MensagemExemplo>("meu_topico_pub", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "meu_topico_sub", 10, std::bind(&SimpleNode::topic_callback, this, std::placeholders::_1));

        // Servidor de serviço (oferecido por este nó)
        service_server_ = this->create_service<template_ros_2_package::srv::ServicoExemplo>("meu_servico",
                                                                                            std::bind(&SimpleNode::service_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Cliente de serviço (para chamar o serviço do LifecycleNode)
        service_client_ = this->create_client<template_ros_2_package::srv::ServicoExemplo>("lifecycle_service");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&SimpleNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "SimpleNode foi inicializado com sucesso!");
        RCLCPP_INFO(this->get_logger(), "Parâmetro 'meu_parametro_string': %s", meu_parametro_string_.c_str());
    }
    /*//}*/

    /* timer_callback() //{ */
    void SimpleNode::timer_callback()
    {
        // Ação 1: Publicar uma mensagem
        auto message = template_ros_2_package::msg::MensagemExemplo();
        message.detalhe = "Olá, mundo! O parâmetro é: " + meu_parametro_string_;
        message.valor = 123;
        publisher_->publish(message);

        // Ação 2: Chamar o serviço do LifecycleNode
        call_service();
    }
    /*//}*/

    /* topic_callback() //{ */
    void SimpleNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Recebi no meu_topico_sub: '%s'", msg->data.c_str());
    }
    /*//}*/

    /* service_callback() //{ */
    void SimpleNode::service_callback(
        const std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Request> request,
        std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Recebi uma requisição de serviço: '%s'", request->pergunta.c_str());
        response->sucesso = true;
        response->resposta = "Olá, cliente! Esta é a resposta do SimpleNode.";
    }
    /*//}*/

    /* call_service() //{ */
    void SimpleNode::call_service()
    {
        // Aguarda o serviço ficar disponível por 1 segundo.
        if (!service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            // Se o LifecycleNode não estiver ATIVO, o serviço não estará disponível.
            RCLCPP_WARN(this->get_logger(), "Serviço 'lifecycle_service' não disponível. O LifecycleNode está ativo?");
            return;
        }

        // Cria a requisição
        auto request = std::make_shared<template_ros_2_package::srv::ServicoExemplo::Request>();
        request->pergunta = "SimpleNode pergunta: tudo bem por aí?";

        // Envia a requisição de forma assíncrona
        auto future_result = service_client_->async_send_request(request,
                                                                 // Callback para processar a resposta quando ela chegar
                                                                 [this](rclcpp::Client<template_ros_2_package::srv::ServicoExemplo>::SharedFuture future)
                                                                 {
                                                                     auto result = future.get();
                                                                     RCLCPP_INFO(this->get_logger(), "Resposta do 'lifecycle_service' recebida: '%s' (Sucesso: %d)", result->resposta.c_str(), result->sucesso);
                                                                 });
    }
    /*//}*/

} // namespace template_ros_2_package