#ifndef IMAGE_READER_NODE_HPP_
#define IMAGE_READER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include "rclcpp/serialization.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <bag_reading_cpp/utils.hpp>

class ImageReaderNode : public rclcpp::Node {
public:
    ImageReaderNode(const std::string& bag_file_path);

    ~ImageReaderNode();

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void timerCallback();

    void initializeOpenCL();

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

    // OpenCL variables
    cl::Context context;
    cl::CommandQueue queue;
    cl::Device device;
    cl_int error;

    // Kernel variables
    cl::Kernel krnl_;
    std::string kerneldir = get_current_dir() + "/kernels/copy.cl";
    std::string kernelcode = readFile(kerneldir);

    // Image variables
    int width;
    int height;
    std::string output_dir = get_current_dir() + "/output_images/";
};

#endif // IMAGE_READER_NODE_HPP_
