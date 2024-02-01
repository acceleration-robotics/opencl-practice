#include <bag_reading_cpp/image_proc.hpp>
#include <chrono>

using namespace std::chrono_literals;

CLImageProc::CLImageProc(const std::string& bag_file_path)
    : Node("image_reader_node"), bag_file_path_(bag_file_path) {
    
    std::cout << "Enter scaling factor : \n";
    std::cin >> scale;
    initializeOpenCL();
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
    
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
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
    RCLCPP_INFO(get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
    width = msg->width;
    height = msg->height;
    resize_width = scale * width;
    resize_height = scale * height;
    cv::Mat image = cv_bridge::toCvCopy(msg, "rgba8")->image;

    const cl::ImageFormat format(CL_BGRA, CL_UNORM_INT8);
    cl::Image2D i_img(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, format, width, height, 0, &image.data[0], &error);
    assert(error == CL_SUCCESS);
    std::cout << "buffer create : " << error << std::endl;
    cl::Image2D o_img(context, CL_MEM_WRITE_ONLY, format, resize_width, resize_height, 0, NULL, &error);
    assert(error == CL_SUCCESS);
    error = krnl_.setArg(0, i_img);
    assert(error == CL_SUCCESS);
    error = krnl_.setArg(1, o_img);
    assert(error == CL_SUCCESS);
    error = krnl_.setArg(2, scale);
    assert(error == CL_SUCCESS);
    queue.enqueueNDRangeKernel(krnl_, cl::NullRange, cl::NDRange(resize_width, resize_height), cl::NullRange);
    cl::size_t<3> origin;
    cl::size_t<3> size;
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
    size[0] = resize_width;
    size[1] = resize_height;
    size[2] = 1;
    unsigned char * tmp = new unsigned char[resize_width * resize_height * 4];
    error = queue.enqueueReadImage(o_img, CL_TRUE, origin, size, 0, 0, tmp);
    // cv::Mat output_image(scale * height, scale * width, CV_8UC4, std::move(tmp));
    cv::Mat output_image(scale * height, scale * width, CV_8UC4);
    std::cout << error;
    std::cout << "buffer read : " << error << std::endl;
    for (int y = 0; y < output_image.rows; ++y) {
        for (int x = 0; x < output_image.cols; ++x) {
            for (int c = 0; c < 4; ++c) {
                output_image.at<cv::Vec4b>(y, x)[c] = tmp[y * output_image.cols * 4 + x * 4 + c];
            }
        }
    }
    delete[] tmp;
    if (output_image.empty()) {
        std::cerr << "Error: Could not load the image." << std::endl;
    }
    queue.finish();
    queue.flush();
    assert(error == CL_SUCCESS);

    // Save the image using OpenCV
    std::string filename = "/home/user/DEVELOPMENT/ros_ws/saved_images/resize_image_" + std::to_string(image_count_) + ".png";
    cv::imwrite(filename, output_image);
    image_count_++;
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

    if (next_msg->topic_name == "/camera/image_raw") {
        sensor_msgs::msg::Image::UniquePtr ros_msg = std::make_unique<sensor_msgs::msg::Image>();
        image_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
        image_publisher_->publish(std::move(ros_msg));
        RCLCPP_INFO(this->get_logger(), "Published Image Message");
    }


    break;
    }
}

void CLImageProc::initializeOpenCL() {
    std::vector<cl::Platform> platforms;
	cl::Platform::get(&platforms);
	if (platforms.size() == 0)
	{
		std::cout << "No OpenCL platforms found" << std::endl;
		exit(1);
	}
	std::vector<cl::Device> devices;
	platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
	// device = devices[0];
    device = std::move(devices[0]);
	std::cout << "Using device: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
	std::cout << "Using platform: " << platforms[0].getInfo<CL_PLATFORM_NAME>() << std::endl;
    cl::Context ctx(device);
    context = std::move(ctx);
    cl::CommandQueue q(context, device, 0, NULL);
    queue = std::move(q);
    // context = ctx; // Copy assignment operator is defined for context
    // queue = q; // Copy assignment operator is defined for queue
    cl::Program::Sources sources;
    sources.push_back({resize_kernelcode.c_str(), resize_kernelcode.length()+1});

    // Create OpenCL program to link source code to context
    cl::Program p(context, sources);
    program = std::move(p);

    // Compile OpenCL code in execution time
    if (program.build({device}) != CL_SUCCESS) {
        std::cout << "Error building program " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;
        exit(1);
    } else {
        std::cout << "Program compiled succesfully!" << std::endl;
    }
    cl::Kernel resize_kernel(program, "resize", &error);
    assert(error == CL_SUCCESS);
    krnl_ = resize_kernel;

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
