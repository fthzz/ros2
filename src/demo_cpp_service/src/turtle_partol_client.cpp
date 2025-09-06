#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "chapt4_interfaces/srv/partol.hpp"

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

};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtlePartolClientNode>("turtle_partol_client");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
