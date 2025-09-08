#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp> //提供消息接口
#include <tf2/LinearMath/Quaternion.h> //四元数类
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> //四元数与消息的转换
#include <tf2_ros/transform_broadcaster.h> //坐标容器
#include <chrono>

using namespace std::chrono_literals;


class TFBroadcasterNode : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    TFBroadcasterNode() : Node("tf_broadcaster")
    {
        this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        this->timer_ = this->create_wall_timer(
            100ms,
            std::bind(&TFBroadcasterNode::publish_tf, this)
        );
    }

    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 30*M_PI/180.0); // Roll, Pitch, Yaw
        transform.transform.rotation = tf2::toMsg(q);

        this->tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFBroadcasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
