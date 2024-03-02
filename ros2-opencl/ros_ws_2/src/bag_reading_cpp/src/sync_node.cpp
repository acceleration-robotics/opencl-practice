#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace std::chrono_literals;

class SyncNode : public rclcpp::Node
{
public:
    SyncNode(const std::string &bag_file) : Node("sync_node")
    {
        rclcpp::QoS qos(10);
        auto rmw_qos_profile = qos.get_rmw_qos_profile();
        rosbag_reader_.open(bag_file);

        image_sub_.subscribe(this, "/presync/image_raw", rmw_qos_profile);
        info_sub_.subscribe(this, "/presync/camera_info", rmw_qos_profile);
        
        
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/presync/image_raw", qos);
        info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/presync/camera_info", qos);

        sync_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/sync/image_raw", qos);
        sync_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/sync/camera_info", qos);

        timer_ = this->create_wall_timer(
        100ms, std::bind(&SyncNode::timerCallback, this));

        syncApproximate.reset(new message_filters::Synchronizer<approximate_policy>(approximate_policy(10), image_sub_, info_sub_));
        syncApproximate->registerCallback(&SyncNode::syncCallback, this);
    }

private:

    void timerCallback() {
        RCLCPP_INFO(this->get_logger(), "In timerCallback");
        if (!rosbag_reader_.has_next())
        { // Check if there are any remaining messages
            RCLCPP_INFO(this->get_logger(), "No more messages in the bag file. Exiting...");
            rclcpp::shutdown(); // Exit the node
            return;
        }
        while (rosbag_reader_.has_next())
        {
            std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_msg = rosbag_reader_.read_next();

            rclcpp::SerializedMessage serialized_msg(*next_msg->serialized_data);

            if (next_msg->topic_name == "/camera/camera_info")
            {
                sensor_msgs::msg::CameraInfo::UniquePtr ros_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
                info_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
                info_publisher_->publish(std::move(ros_msg));
                RCLCPP_INFO(this->get_logger(), "Published CameraInfo Message");
            }
            else if (next_msg->topic_name == "/camera/image_raw")
            {
                sensor_msgs::msg::Image::UniquePtr ros_msg = std::make_unique<sensor_msgs::msg::Image>();
                image_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
                image_publisher_->publish(std::move(ros_msg));
                RCLCPP_INFO(this->get_logger(), "Published Image Message");
            }

            break;
        }
    }

    void syncCallback(const sensor_msgs::msg::Image& img_msg, const sensor_msgs::msg::CameraInfo& info_msg) {
        RCLCPP_INFO(this->get_logger(), "In syncCallBack");
        sync_image_publisher_->publish(img_msg);
        sync_info_publisher_->publish(info_msg);
        // const sensor_msgs::msg::Image::SharedPtr img_msg, const sensor_msgs::msg::CameraInfo::SharedPtr info_msg
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
   
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sync_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr sync_info_publisher_;
    rosbag2_cpp::Reader rosbag_reader_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization_;
    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> info_serialization_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> approximate_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> syncApproximate;
};  

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: ros2 run bag_reading_cpp sync_node <bag_file>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
