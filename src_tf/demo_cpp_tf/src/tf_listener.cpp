#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp> //提供消息接口
#include <tf2/LinearMath/Quaternion.h> //四元数类
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> //四元数与消息的转换
#include <tf2_ros/transform_listener.h> //坐标容器
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <chrono>

using namespace std::chrono_literals;


class TFListenerNode : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
public:
    TFListenerNode() : Node("tf_listener")
    {
        this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        this->timer_ = this->create_wall_timer(
            1s,
            std::bind(&TFListenerNode::get_transform, this)
        );
    }

    void get_transform()
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = this->tf_buffer_->lookupTransform("base_link", "target_point", this->get_clock()->now(),
            rclcpp::Duration::from_seconds(1.0));
            RCLCPP_INFO(this->get_logger(), "Translation: [%.2f, %.2f, %.2f]", 
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);

            double roll, pitch, yaw;
            tf2::getEulerYPR(transform.transform.rotation, yaw, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "Rotation: [Roll: %.2f, Pitch: %.2f, Yaw: %.2f]", 
                roll, pitch, yaw);
            
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
