#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp> //提供消息接口
#include <tf2/LinearMath/Quaternion.h> //四元数类
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> //四元数与消息的转换
#include <tf2_ros/static_transform_broadcaster.h> //静态坐标容器


class StaticTFBroadcasterNode : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

public:
    StaticTFBroadcasterNode() : Node("static_tf_broadcaster")
    {
        this->static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);//这里输入节点this作为参数的作用是：函数会自动创建一个发布者取发布一个话题tf_static
        publish_tf();
    }

    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "target_point";

        transform.transform.translation.x = 5.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 60*M_PI/180.0); // Roll, Pitch, Yaw
        transform.transform.rotation = tf2::toMsg(q);

        this->static_tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
