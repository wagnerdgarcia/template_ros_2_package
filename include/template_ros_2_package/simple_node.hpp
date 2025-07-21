#ifndef TEMPLATE_ROS_2_PACKAGE__SIMPLE_NODE_HPP_
#define TEMPLATE_ROS_2_PACKAGE__SIMPLE_NODE_HPP_

/* includes //{ */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "template_ros_2_package/msg/mensagem_exemplo.hpp"
#include "template_ros_2_package/srv/servico_exemplo.hpp"

/*//}*/

namespace template_ros_2_package
{
    /**
     * @class SimpleNode
     * @brief Um nó ROS 2 simples que demonstra as funcionalidades básicas de comunicação.
     */
    class SimpleNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Construtor da classe SimpleNode.
         * @param options Opções do nó ROS 2.
         */
        /* SimpleNode() //{ */
        explicit SimpleNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        /*//}*/

    private:
        /* HELPER FUNCTIONS //{ */

        /**
         * @brief Função para realizar uma chamada de serviço assíncrona.
         */
        /* call_service() //{ */
        void call_service();
        /*//}*/

        /*//}*/

        /* CALLBACKS //{ */

        /**
         * @brief Callback executado periodicamente pelo timer.
         */
        /* timer_callback() //{ */
        void timer_callback();
        /*//}*/

        /**
         * @brief Callback executado ao receber uma mensagem no tópico de subscrição.
         * @param msg A mensagem recebida.
         */
        /* topic_callback() //{ */
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
        /*//}*/

        /**
         * @brief Callback executado ao receber uma requisição de serviço.
         * @param request A requisição recebida.
         * @param response A resposta a ser enviada.
         */
        /* service_callback() //{ */
        void service_callback(const std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Request> request,
                              std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Response> response);
        /*//}*/

        /*//}*/

        /* PARÂMETROS //{ */
        std::string meu_parametro_string_; ///< Parâmetro do tipo string lido do arquivo YAML.
        double meu_parametro_double_;      ///< Parâmetro do tipo double lido do arquivo YAML.
        /*//}*/

        /* PUBLISHERS //{ */
        rclcpp::Publisher<template_ros_2_package::msg::MensagemExemplo>::SharedPtr publisher_;
        /*//}*/

        /* SUBSCRIBERS //{ */
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        /*//}*/

        /* SERVICES //{ */
        rclcpp::Service<template_ros_2_package::srv::ServicoExemplo>::SharedPtr service_server_;
        rclcpp::Client<template_ros_2_package::srv::ServicoExemplo>::SharedPtr service_client_;
        /*//}*/

        /* TIMERS //{ */
        rclcpp::TimerBase::SharedPtr timer_;
        /*//}*/
    };

} // namespace template_ros_2_package

#endif // TEMPLATE_ROS_2_PACKAGE__SIMPLE_NODE_HPP_