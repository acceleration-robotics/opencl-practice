#include <bag_reading_cpp/image_proc.hpp>
#include <chrono>

using namespace std::chrono_literals;

CLImageProc::CLImageProc(const std::string& bag_file_path)
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
    
    info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            getCameraInfo(msg);
        });

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&CLImageProc::timerCallback, this));

    // Open the bag file for reading with default options
    rosbag_reader_ = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
    rosbag_reader_->open(storage_options, converter_options);  // Provide an empty ConverterOptions
}

CLImageProc::~CLImageProc() {
    // No need to explicitly close the bag file, smart pointer handles it
    rosbag_reader_->reset();
}

void CLImageProc::processImage(const sensor_msgs::msg::Image::SharedPtr msg) { 

    auto ogstart = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
    width = msg->width;
    height = msg->height;
    cv::Mat image = cv_bridge::toCvCopy(msg, "rgba8")->image;
    cv::Mat rect;
    cv::Mat res;
    int scale = 4;
    auto start = std::chrono::high_resolution_clock::now();
    // model_.rectifyImage(image, rect, cv::INTER_LINEAR);
    auto stop = std::chrono::high_resolution_clock::now();
    auto rect_duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = std::chrono::high_resolution_clock::now();
    cv::resize(image, res, cv::Size(scale * width, scale * height), cv::INTER_LINEAR);
    stop = std::chrono::high_resolution_clock::now();
    auto res_duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    // Save the image using OpenCV
    std::string filename = "/root/DEVELOPMENT/opencl-test-codes/ros2-opencl/ros_ws/saved_images/resize_cpu_image_" + std::to_string(image_count_) + ".png";
    cv::imwrite(filename, res);
    image_count_++;

    stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - ogstart);
    std::string new_line = std::to_string(rect_duration.count()) + "," + std::to_string(res_duration.count()) + "," +
    std::to_string(duration.count()) + ",\n";
    std::ofstream myfile;
    myfile.open("/root/DEVELOPMENT/opencl-test-codes/ros2-opencl/ros_ws/analyses/resize_cpu.csv", std::ios::app);
    myfile << new_line;
}

void CLImageProc::getCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) { 
    RCLCPP_INFO(get_logger(), "Received Camera Info");
    model_.fromCameraInfo(msg);
    // Extract Intrinsic Camera Parameters
    image_matrices[0] = msg->k[0]; // fx
    image_matrices[1] = msg->k[4]; // fy
    image_matrices[2] = msg->k[2]; // cx
    image_matrices[3] = msg->k[5]; // cy

    // Extract Distortion Coefficients
    image_matrices[4] = msg->d[0]; // k1
    image_matrices[5] = msg->d[1]; // k2
    image_matrices[6] = msg->d[4]; // k3
    image_matrices[7] = msg->d[2]; // p1
    image_matrices[8] = msg->d[3]; // p2

    // Extract Rectification Matrix
    image_matrices[9] = msg->r[0];
    image_matrices[10] = msg->r[1];
    image_matrices[11] = msg->r[2];
    image_matrices[12] = msg->r[3];
    image_matrices[13] = msg->r[4];
    image_matrices[14] = msg->r[5];
    image_matrices[15] = msg->r[6];
    image_matrices[16] = msg->r[7];
    image_matrices[17] = msg->r[8];

    // Extract Projection Matrix
    image_matrices[18] = msg->p[0];
    image_matrices[19] = msg->p[1];
    image_matrices[20] = msg->p[2];
    image_matrices[21] = msg->p[3];
    image_matrices[22] = msg->p[4];
    image_matrices[23] = msg->p[5];
    image_matrices[24] = msg->p[6];
    image_matrices[25] = msg->p[7];
    image_matrices[26] = msg->p[8];
    image_matrices[27] = msg->p[9];
    image_matrices[28] = msg->p[10];
    image_matrices[29] = msg->p[11];
}

void CLImageProc::timerCallback() {
    if (!rosbag_reader_->has_next()) { // Check if there are any remaining messages
        RCLCPP_INFO(this->get_logger(), "No more messages in the bag file. Exiting...");
        rclcpp::shutdown(); // Exit the node
        return;
    }
    while (rosbag_reader_->has_next()) {
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_msg = rosbag_reader_->read_next();

    rclcpp::SerializedMessage serialized_msg(*next_msg->serialized_data);

    if (next_msg->topic_name == "/camera/camera_info") {
        sensor_msgs::msg::CameraInfo::UniquePtr ros_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
        info_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
        info_publisher_->publish(std::move(ros_msg));
        RCLCPP_INFO(this->get_logger(), "Published CameraInfo Message");
    } else if (next_msg->topic_name == "/camera/image_raw") {
        sensor_msgs::msg::Image::UniquePtr ros_msg = std::make_unique<sensor_msgs::msg::Image>();
        image_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
        image_publisher_->publish(std::move(ros_msg));
        RCLCPP_INFO(this->get_logger(), "Published Image Message");
    }


    break;
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <bag_file_path>\n", argv[0]);
        return 1;
    }
    auto start = std::chrono::high_resolution_clock::now();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CLImageProc>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Program Execution Time (ms) : " << duration.count() << std::endl;

    return 0;
}
