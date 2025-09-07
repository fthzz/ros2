#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "chapt4_interfaces/srv/partol.hpp"
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

using SetParameters = rcl_interfaces::srv::SetParameters;
using namespace std::chrono_literals;
using Partol = chapt4_interfaces::srv::Partol;


class TurtlePartolClientNode : public rclcpp::Node{
private:
    rclcpp::Client<Partol>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    explicit TurtlePartolClientNode(const std::string &node_name) : Node(node_name) {
        client_ = this->create_client<Partol>("partol");
        timer_ = this->create_wall_timer(5s, [&] () -> void {
            while (!client_->wait_for_service(1s)){
                RCLCPP_WARN(this->get_logger(),"等待服务端启动...");
            }
            auto request = std::make_shared<Partol::Request>();
            request->target_x = rand() % 11; //0-10
            request->target_y = rand() % 11; //0-10
            RCLCPP_INFO(this->get_logger(),"请求巡逻点:(%f,%f)",request->target_x,request->target_y);

            this -> client_ -> async_send_request(request, [&](rclcpp::Client<Partol>::SharedFuture future) -> void {
                auto response = future.get();
                if (response->result == Partol::Response::SUCCESS){
                    RCLCPP_INFO(this->get_logger(),"巡逻点设置成功");
                }
                else{
                    RCLCPP_ERROR(this->get_logger(),"巡逻点设置失败");
                }
            });
        });
    }
        //写这个函数的主要作用是之后要修改参数，就方便写外接接口函数了
    SetParameters::Response::SharedPtr call_set_parameter(const rcl_interfaces::msg::Parameter &param){
        auto param_client = this->create_client<SetParameters>("set_parameters");
        while (!param_client->wait_for_service(1s)){
            RCLCPP_WARN(this->get_logger(),"等待服务端启动...");
        }
        auto request = std::make_shared<SetParameters::Request>();
        request->parameters.push_back(param); //这个parameters是一个vector，这个操作是把参数加入vector

        auto result = param_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS){
            return result.get();
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"设置参数失败");
            return nullptr;
        }
    }

    //更新k的值
    void update_service_param_k(double k){
        //1.创建参数对象
        rcl_interfaces::msg::Parameter param;
        param.name = "k";
        //2.构建参数值
        rcl_interfaces::msg::ParameterValue value;
        value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        value.double_value = k;
        param.value = value;
        //3.请求更新参数并处理
        auto response = call_set_parameter(param);
        if (response != nullptr){
            for (const auto &result : response->results){
                if (result.successful){
                    RCLCPP_INFO(this->get_logger(),"参数k设置成功");
                }
                else{
                    RCLCPP_ERROR(this->get_logger(),"参数k设置失败");
                }
            }
        }
    }

};


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtlePartolClientNode>("turtle_partol_client");
    node->update_service_param_k(4.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
