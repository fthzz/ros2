#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <turtlesim/msg/pose.hpp>
#include "chapt4_interfaces/srv/partol.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp> //用于参数设置的消息接口

using namespace std::chrono_literals;
using Partol = chapt4_interfaces::srv::Partol;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class TurtleControlNode : public rclcpp::Node{
private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_; //参数回调函数的智能指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; //发布者的智能指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_; //订阅者的智能指针
    rclcpp::Service<Partol>::SharedPtr service_;
    double target_x_ = 1.0;
    double target_y_ = 1.0;
    double k_ = 1.0; //比例增益
    double max_linear_speed_ = 3.0; //线速度上限
public:
    explicit TurtleControlNode(const std::string &node_name):Node(node_name){
        this->declare_parameter("k",1.0);
        this->declare_parameter("max_linear_speed",3.0);
        this->get_parameter("k",k_);
        this->get_parameter("max_linear_speed",max_linear_speed_);

        // this->set_parameter(rclcpp::Parameter("k",2.0)); //修改参数值

        param_callback_handle_ = this->add_on_set_parameters_callback(
            [&](const std::vector<rclcpp::Parameter> &parameters) -> SetParametersResult {
                SetParametersResult result;
                result.successful = true;

                for (const auto &parameter : parameters){
                    if (parameter.get_name() == "k"){
                        k_ = parameter.as_double(); //将参数值转换为double类型
                    }
                    else if (parameter.get_name() == "max_linear_speed"){
                        max_linear_speed_ = parameter.as_double();
                    }
                }
                RCLCPP_INFO(this->get_logger(),"更新参数：k=%f,max_linear_speed=%f",k_,max_linear_speed_);
                return result;
            }
        );


        service_ = this->create_service<Partol>("partol", [&](const Partol::Request::SharedPtr request,
        Partol::Response::SharedPtr response) -> void {
            if (request->target_x < 0.0 || request->target_x > 11.0 ||
               request->target_y < 0.0 || request->target_y > 11.0){
                RCLCPP_ERROR(this->get_logger(),"目标点设置错误，x和y的范围是[0.0,11.0]");
                response->result = Partol::Response::FAIL;
            }
            else{
                this->target_x_ = request->target_x;
                this->target_y_ = request->target_y;
                response->result = Partol::Response::SUCCESS;
            }
        });

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",10,std::bind(&TurtleControlNode::pose_callback,this,std::placeholders::_1)
        );  //因为这个回调函数有一个输入参数，所以需要一个占位符
    }

    //获取数据时，获取的是一个智能指针
    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose){
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(this->get_logger(),"当前坐标:(%f,%f)",current_x,current_y);

        auto distance = std::sqrt(std::pow(target_x_ - current_x,2) + std::pow(target_y_ - current_y,2));
        auto angle = std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

        auto msg = geometry_msgs::msg::Twist();
        if (distance > 0.1) {
            if (fabs(angle) > 0.2 ) msg.angular.z = fabs(angle);
            else msg.linear.x = k_ * distance;
        }

        if (msg.linear.x > max_linear_speed_) msg.linear.x = max_linear_speed_;

        publisher_->publish(msg);
    }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleControlNode>("turtle_control");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
