#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include "rclcpp/serialization.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_bag_reader.hpp>
#include <CL/cl2.hpp>

#include <chrono>

using namespace std::chrono_literals;

class ImageReaderNode : public rclcpp::Node {
public:
    ImageReaderNode(const std::string& bag_file_path)
        : Node("image_reader_node"), bag_file_path_(bag_file_path) {

        rosbag2_cpp::StorageOptions storage_options;
        storage_options.uri = bag_file_path_;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        // Create a subscriber for the image topic
        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                processImage(msg);
            });
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        timer_ = this->create_wall_timer(
          100ms, std::bind(&ImageReaderNode::timer_callback, this));

        // Open the bag file for reading with default options
        rosbag_reader_ = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
        rosbag_reader_->open(storage_options, converter_options);  // Provide an empty ConverterOptions
    }

    ~ImageReaderNode() {
        // No need to explicitly close the bag file, smart pointer handles it
        rosbag_reader_->reset();
    }

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Implement your image processing logic here
        // You can access the image data using msg->data
        RCLCPP_INFO(get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
        // Decode compressed image using OpenCV
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        // cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        // Save the image using OpenCV
        std::string filename = "/home/user/DEVELOPMENT/ros_ws/saved_images/image_" + std::to_string(image_count_) + ".png";
        cv::imwrite(filename, image);
        image_count_++;
    }

    void timer_callback() {
        if (!rosbag_reader_->has_next()) { // Check if there are any remaining messages
            RCLCPP_INFO(this->get_logger(), "No more messages in the bag file. Exiting...");
            rclcpp::shutdown(); // Exit the node
            return;
        }
        while (rosbag_reader_->has_next()) {
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_msg = rosbag_reader_->read_next();

        if (next_msg->topic_name != "/camera/image_raw") { // Updated topic
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(*next_msg->serialized_data);
        sensor_msgs::msg::Image::UniquePtr ros_msg = std::make_unique<sensor_msgs::msg::Image>();

        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        publisher_->publish(std::move(ros_msg));
        std::cout << "Published image message\n";

        break;
      }
    }

    // ROS 2 subscriber for the image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

    // ROS bag reader
    std::shared_ptr<rosbag2_cpp::readers::SequentialReader> rosbag_reader_;

    // Path to the bag file
    std::string bag_file_path_;

    // Serializer
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;

    // Image publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Image count
    int image_count_ = 0;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <bag_file_path>\n", argv[0]);
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageReaderNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
