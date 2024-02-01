#ifndef IMAGE_PROC_HPP_
#define IMAGE_PROC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include "rclcpp/serialization.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <bag_reading_cpp/utils.hpp>

class CLImageProc : public rclcpp::Node {
public:
    CLImageProc(const std::string& bag_file_path);

    CLImageProc(const std::string& bag_file_path, const int scaling_factor);

    ~CLImageProc();

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void getCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void timerCallback();

    void initializeOpenCL();

    // ROS 2 subscriber for the image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

    // ROS 2 subscriber for the camera info topic
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscriber_;

    // ROS bag reader
    std::shared_ptr<rosbag2_cpp::readers::SequentialReader> rosbag_reader_;

    // Path to the bag file
    std::string bag_file_path_;

    // Serializer
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization_;
    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> info_serialization_;

    // Image publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    // Info publisher
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Image count
    int image_count_ = 0;
    int info_count_ = 0;
    mutable bool next = false;

    // OpenCL variables
    cl::Context context;
    cl::CommandQueue r_queue, w_queue, k_queue; // Read queue, Write queue, Kernel queue
    cl::Device device;
    cl::Program program;
    cl_int error;

    // Kernel variables
    cl::Kernel krnl_1, krnl_2;
    std::string kerneldir = get_current_dir() + "/kernels/copy.cl";
    std::string kernelcode = readFile(kerneldir);

    // Pinhole camera model (for CPU implementation)
    image_geometry::PinholeCameraModel model_;

    // Image variables
    int width;
    int height;
    int resize_width;
    int resize_height;
    int scale = 4;
    std::string output_dir = get_current_dir() + "/output_images/";

    // Image rectification variables
    float image_matrices[30] = {0.0}; // fx, fy, cx, cy, k1, k2, k3, p1,
                                      // p2, r11, r12, r13, r21, r22, r23, 
                                      // r31, r32, r33, p11, p12, p13, p14, 
                                      // p21, p22, p23, p24, p31, p32, p33, p34;
};

#endif // IMAGE_BAG_READER_HPP_
