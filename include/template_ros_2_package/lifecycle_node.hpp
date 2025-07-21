#ifndef TEMPLATE_ROS_2_PACKAGE__LIFECYCLE_NODE_HPP_
#define TEMPLATE_ROS_2_PACKAGE__LIFECYCLE_NODE_HPP_

/* includes //{ */

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "template_ros_2_package/msg/mensagem_exemplo.hpp"
#include "template_ros_2_package/srv/servico_exemplo.hpp"

/*//}*/

/* define //{*/

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/*//}*/

namespace template_ros_2_package
{
    /**
     * @class LifecycleNode
     * @brief Um nó ROS 2 que implementa o ciclo de vida para gerenciar seus recursos.
     */
    class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        /**
         * @brief Construtor da classe LifecycleNode.
         * @param options Opções do nó ROS 2.
         */
        /* LifecycleNode() //{ */
        explicit LifecycleNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        /*//}*/

    private:
        /* LIFECYCLE TRANSITIONS //{ */

        /**
         * @brief Chamado na transição para o estado 'Configuring'. Orquestra a alocação de recursos.
         * @param state O estado anterior.
         * @return CallbackReturn::SUCCESS ou CallbackReturn::FAILURE.
         */
        /* on_configure() //{ */
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Chamado na transição para o estado 'Activating'. Ativa os recursos.
         * @param state O estado anterior.
         * @return CallbackReturn::SUCCESS ou CallbackReturn::FAILURE.
         */
        /* on_activate() //{ */
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Chamado na transição para o estado 'Deactivating'. Desativa os recursos.
         * @param state O estado anterior.
         * @return CallbackReturn::SUCCESS ou CallbackReturn::FAILURE.
         */
        /* on_deactivate() //{ */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Chamado na transição para o estado 'CleaningUp'. Libera os recursos.
         * @param state O estado anterior.
         * @return CallbackReturn::SUCCESS ou CallbackReturn::FAILURE.
         */
        /* on_cleanup() //{ */
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Chamado na transição para 'Shutdown'. Limpeza final.
         * @param state O estado anterior.
         * @return CallbackReturn::SUCCESS ou CallbackReturn::FAILURE.
         */
        /* on_shutdown() //{ */
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /*//}*/

        /* HELPER FUNCTIONS FOR CONFIGURATION //{ */

        /**
         * @brief Carrega os parâmetros a partir do arquivo de configuração.
         */
        /* getParameters() //{ */
        void getParameters();
        /*//}*/

        /**
         * @brief Configura os publishers e subscribers para comunicação.
         */
        /* configPubSub() //{ */
        void configPubSub();
        /*//}*/

        /**
         * @brief Configura os timers para execução periódica.
         */
        /* configTimers() //{ */
        void configTimers();
        /*//}*/

        /**
         * @brief Configura os servidores e clientes de serviço.
         */
        /* configServices() //{ */
        void configServices();
        /*//}*/

        /**
         * @brief Função para realizar uma chamada de serviço assíncrona.
         */
        /* call_service() //{ */
        void call_service();
        /*//}*/

        /*//}*/

        /* COMMUNICATION CALLBACKS //{ */

        /**
         * @brief Callback executado periodicamente pelo timer.
         */
        /* timer_callback() //{ */
        void timer_callback();
        /*//}*/

        /**
         * @brief Callback para o tópico de subscrição.
         * @param msg A mensagem recebida.
         */
        /* topic_callback() //{ */
        void topic_callback(const std_msgs::msg::String::SharedPtr msg);
        /*//}*/

        /**
         * @brief Callback para o servidor de serviço.
         * @param request A requisição do cliente.
         * @param response A resposta a ser enviada.
         */
        /* service_callback() //{ */
        void service_callback(const std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Request> request,
                              std::shared_ptr<template_ros_2_package::srv::ServicoExemplo::Response> response);
        /*//}*/

        /*//}*/

        /* NODE RESOURCES //{ */
        std::string parametro_lifecycle_; ///< Parâmetro lido do arquivo YAML.
        bool is_active_ = false;          ///< Flag para controlar o estado operacional.

        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<template_ros_2_package::msg::MensagemExemplo>> publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Service<template_ros_2_package::srv::ServicoExemplo>::SharedPtr service_server_;
        rclcpp::Client<template_ros_2_package::srv::ServicoExemplo>::SharedPtr service_client_;
        rclcpp::TimerBase::SharedPtr timer_;
        /*//}*/
    };

} // namespace template_ros_2_package

#endif // TEMPLATE_ROS_2_PACKAGE__LIFECYCLE_NODE_HPP_