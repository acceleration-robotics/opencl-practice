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

namespace image_proc
{

class RectifyCL : public rclcpp::Node {
public:
    explicit RectifyCL(const rclcpp::NodeOptions &);

    // CLImageProc(const int scaling_factor);

    ~RectifyCL();

protected:
    image_transport::CameraPublisher pub_image_;
    image_transport::CameraSubscriber sub_image_;

private:

    void imageCb(sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

    void getCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void initializeOpenCL();

    size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg);
    size_t get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

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
    cl::Kernel krnl_2;
    std::string rectify_kerneldir = get_current_dir() + "/kernels/copy.cl";
    std::string rectify_kernelcode = readFile(rectify_kerneldir);
    std::string resize_kerneldir = get_current_dir() + "/kernels/copy.cl";
    std::string resize_kernelcode = readFile(resize_kerneldir);

    // Pinhole camera model (for CPU implementation)
    image_geometry::PinholeCameraModel model_;

    // Image variables
    int queue_size_;
    int interpolation;
    bool use_scale_ = false;
    bool profile_;
    double scale_height_ = 1.0;
    double scale_width_ = 1.0;
    int height_ = -1;
    int width_ = -1;


    int width;
    int height;
    int resize_width;
    int resize_height;
    int scale = 1;
    std::string output_dir = get_current_dir() + "/output_images/";

    // Image rectification variables
    float image_matrices[30] = {0.0}; // fx, fy, cx, cy, k1, k2, k3, p1,
                                      // p2, r11, r12, r13, r21, r22, r23, 
                                      // r31, r32, r33, p11, p12, p13, p14, 
                                      // p21, p22, p23, p24, p31, p32, p33, p34;
};

}  // namespace image_proc

#endif // IMAGE_PROC_HPP_
