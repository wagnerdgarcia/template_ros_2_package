#include "template_ros_2_package/lifecycle_node.hpp"

namespace template_ros_2_package
{

    /* LifecycleNode() //{ */
    LifecycleNode::LifecycleNode(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("meu_lifecycle_node", options)
    {
        RCLCPP_INFO(get_logger(), "LifecycleNode criado (estado não configurado).");
    }
    /*//}*/

    /* LIFECYCLE TRANSITIONS //{ */

    /* on_configure() //{ */
    CallbackReturn LifecycleNode::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Iniciando a transição [on_configure]...");

        getParameters();
        configPubSub();
        configTimers();
        configServices();

        RCLCPP_INFO(get_logger(), "Transição [on_configure] concluída com sucesso.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_activate() //{ */
    CallbackReturn LifecycleNode::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[on_activate] Ativando...");
        publisher_->on_activate();
        timer_->reset();
        is_active_ = true;
        RCLCPP_INFO(get_logger(), "Ativação concluída. Nó está operacional.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_deactivate() //{ */
    CallbackReturn LifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[on_deactivate] Desativando...");
        publisher_->on_deactivate();
        timer_->cancel();
        is_active_ = false;
        RCLCPP_INFO(get_logger(), "Desativação concluída.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_cleanup() //{ */
    CallbackReturn LifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[on_cleanup] Limpando...");
        publisher_.reset();
        subscription_.reset();
        service_server_.reset();
        service_client_.reset();
        timer_.reset();
        RCLCPP_INFO(get_logger(), "Limpeza concluída.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_shutdown() //{ */
    CallbackReturn LifecycleNode::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[on_shutdown] Desligando...");
        publisher_.reset();
        subscription_.reset();
        service_server_.reset();
        service_client_.reset();
        timer_.reset();
        RCLCPP_INFO(get_logger(), "Nó desligado.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /*//}*/

    /* HELPER FUNCTIONS FOR CONFIGURATION //{ */

    /* getParameters() //{ */
    void LifecycleNode::getParameters()
    {
        RCLCPP_INFO(get_logger(), "Carregando parâmetros...");
        this->declare_parameter<std::string>("parametro_lifecycle", "default");
        this->get_parameter("parametro_lifecycle", parametro_lifecycle_);
    }
    /*//}*/

    /* configPubSub() //{ */
    void LifecycleNode::configPubSub()
    {
        RCLCPP_INFO(get_logger(), "Configurando publishers e subscribers...");
        publisher_ = this->create_publisher<template_ros_2_package::msg::MensagemExemplo>("lifecycle_topic_pub", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "lifecycle_topic_sub", 10, std::bind(&LifecycleNode::topic_callback, this, std::placeholders::_1));
    }
    /*//}*/

    /* configTimers() //{ */
    void LifecycleNode::configTimers()
    {
        RCLCPP_INFO(get_logger(), "Configurando timers...");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&LifecycleNode::timer_callback, this));
        timer_->cancel(); // O timer só será ativado em on_activate
    }
    /*//}*/

    /* configServices() //{ */
    void LifecycleNode::configServices()
    {
        RCLCPP_INFO(get_logger(), "Configurando serviços...");
        service_server_ = this->create_service<template_ros_2_package::srv::ServicoExemplo>(
            "lifecycle_service", std::bind(&LifecycleNode::service_callback, this, std::placeholders::_1, std::placeholders::_2));
        service_client_ = this->create_client<template_ros_2_package::srv::ServicoExemplo>("lifecycle_service");
    }
    /*//}*/

    /*//}*/

    /* COMMUNICATION CALLBACKS //{ */

    /* timer_callback() //{ */
    void LifecycleNode::timer_callback()
    {
        if (!is_active_)
            return;

        auto msg = template_ros_2_package::msg::MensagemExemplo();
        msg.detalhe = "Mensagem do timer com o parâmetro: " + parametro_lifecycle_;
        msg.valor = this->get_clock()->now().seconds();
        publisher_->publish(msg);

        call_service();
    }
    /*//}*/

    /* topic_callback() //{ */
    void LifecycleNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Mensagem recebida em 'lifecycle_topic_sub' (estado atual: %s): %s",
                    this->get_current_state().label().c_str(), msg->data.c_str());
    }
    /*//}*/

    /* service_callback() //{ */
    void LifecycleNode::service_callback(
        const std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Request> request,
        std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Response> response)
    {
        if (!is_active_)
        {
            RCLCPP_WARN(this->get_logger(), "Serviço chamado enquanto o nó estava inativo. Requisição ignorada.");
            response->sucesso = false;
            response->resposta = "Nó inativo.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Requisição de serviço recebida: '%s'", request->pergunta.c_str());
        response->sucesso = true;
        response->resposta = "Resposta do serviço do nó de ciclo de vida!";
    }
    /*//}*/

    /*//}*/

    /* HELPER FUNCTIONS //{ */

    /* call_service() //{ */
    void LifecycleNode::call_service()
    {
        if (!service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Serviço 'lifecycle_service' não está disponível.");
            return;
        }

        auto request = std::make_shared<template_ros_2_package::srv::ServicoExemplo::Request>();
        request->pergunta = "Podemos conversar?";

        auto future_result = service_client_->async_send_request(request,
                                                                 [this](rclcpp::Client<template_ros_2_package::srv::ServicoExemplo>::SharedFuture future)
                                                                 {
                                                                     auto result = future.get();
                                                                     RCLCPP_INFO(this->get_logger(), "Resposta do serviço recebida: '%s' (Sucesso: %d)", result->resposta.c_str(), result->sucesso);
                                                                 });
    }
    /*//}*/

    /*//}*/

} // namespace template_ros_2_package