#include <bag_reading_cpp/image_bag_reader.hpp>
#include <chrono>

using namespace std::chrono_literals;

ImageReaderNode::ImageReaderNode(const std::string& bag_file_path)
    : Node("image_reader_node"), bag_file_path_(bag_file_path) {

    // initializeOpenCL();
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
        100ms, std::bind(&ImageReaderNode::timerCallback, this));

    // Open the bag file for reading with default options
    rosbag_reader_ = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
    rosbag_reader_->open(storage_options, converter_options);  // Provide an empty ConverterOptions
}

ImageReaderNode::~ImageReaderNode() {
    // No need to explicitly close the bag file, smart pointer handles it
    rosbag_reader_->reset();
}

void ImageReaderNode::processImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    
    RCLCPP_INFO(get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
    width = msg->width;
    height = msg->height;
    const cl::ImageFormat format(CL_BGRA, CL_UNORM_INT8);
    // Decode compressed image using OpenCV
    cv::Mat image = cv_bridge::toCvCopy(msg, "rgba8")->image;
    // cv::Mat output_image(image.size(), image.type());
    // cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    std::vector<cl::Platform> platforms;
	cl::Platform::get(&platforms);
	if (platforms.size() == 0)
	{
		std::cout << "No OpenCL platforms found" << std::endl;
		exit(1);
	}
	std::vector<cl::Device> devices;
	platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
	device = devices[0];
	std::cout << "Using device: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
	std::cout << "Using platform: " << platforms[0].getInfo<CL_PLATFORM_NAME>() << std::endl;
    cl::Context context(device);
    cl::CommandQueue queue(context, device, 0, NULL);
    // context = ctx; // Copy assignment operator is defined for context
    // queue = q; // Copy assignment operator is defined for queue
    cl::Program::Sources sources;
    std::string kernelcode = readFile(kerneldir);
    sources.push_back({kernelcode.c_str(), kernelcode.length()+1});

    // Create OpenCL program to link source code to context
    cl::Program program(context, sources);

    // Compile OpenCL code in execution time
    if (program.build({device}) != CL_SUCCESS) {
        std::cout << "Error building program " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;
        exit(1);
    } else {
        std::cout << "Program compiled succesfully!" << std::endl;
    }
    cl::Kernel copy_kernel(program, "copy", &error);
    assert(error == CL_SUCCESS);
    krnl_ = copy_kernel;

    cl::Image2D i_img(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, format, width, height, 0, &image.data[0], &error);
    assert(error == CL_SUCCESS);
    std::cout << "buffer create : " << error << std::endl;
    cl::Image2D o_img(context, CL_MEM_READ_WRITE, format, width, height, 0, NULL, &error);
    assert(error == CL_SUCCESS);
    error = krnl_.setArg(0, i_img);
    assert(error == CL_SUCCESS);
    error = krnl_.setArg(1, o_img);
    assert(error == CL_SUCCESS);
    // cl::Event event;
    // std::vector<cl::Event> events;
    // events.push_back(event);
    cl_int queueStatus;
    cl_int error = queue.getInfo(CL_QUEUE_PROPERTIES, &queueStatus);
    if (error != CL_SUCCESS) {
        std::cerr << "Error getting queue status: " << error << std::endl;
        // Handle the error as needed
    }

    if (queueStatus & CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE) {
        std::cout << "Out-of-order execution is enabled for this queue." << std::endl;
    } else {
        std::cout << "Out-of-order execution is not enabled for this queue." << std::endl;
    }

    if (queueStatus & CL_QUEUE_PROFILING_ENABLE) {
        std::cout << "Profiling is enabled for this queue." << std::endl;
    } else {
        std::cout << "Profiling is not enabled for this queue." << std::endl;
    }
    queue.enqueueNDRangeKernel(krnl_, cl::NullRange, cl::NDRange(width, height), cl::NullRange);
    // Wait for kernel code execution
    // events[0].wait();
    // queue.finish();
    // queue.flush();
    // Image reading coordinates
    cl::size_t<3> origin;
    cl::size_t<3> size;
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
    size[0] = width;
    size[1] = height;
    size[2] = 1;
    auto tmp = new unsigned char[width * height * 4];
    error = queue.enqueueReadImage(o_img, CL_TRUE, origin, size, 0, 0, tmp);
    // assert(error == CL_SUCCESS);
    std::cout << "buffer read : " << error << std::endl;
    cv::Mat output_image(height, width, CV_8UC4, tmp);
    // std::copy(&tmp[0], &tmp[width*height*4], std::back_inserter(output_image.data));
    // for (int i=0; i<width*height*4; i++) {
    //     std::cout << (int)tmp[i] << "\n";
    // }
    delete[] tmp;
    queue.finish();
    queue.flush();
    assert(error == CL_SUCCESS);

    // Save the image using OpenCV
    std::string filename = "/home/user/DEVELOPMENT/ros_ws/saved_images/image_" + std::to_string(image_count_) + ".png";
    cv::imwrite(filename, output_image);
    image_count_++;
}

void ImageReaderNode::timerCallback() {
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

void ImageReaderNode::initializeOpenCL() {
    std::vector<cl::Platform> platforms;
	cl::Platform::get(&platforms);
	if (platforms.size() == 0)
	{
		std::cout << "No OpenCL platforms found" << std::endl;
		exit(1);
	}
	std::vector<cl::Device> devices;
	platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
	device = devices[0];
	std::cout << "Using device: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
	std::cout << "Using platform: " << platforms[0].getInfo<CL_PLATFORM_NAME>() << std::endl;
    cl::Context ctx(device);
    cl::CommandQueue q(context, device, 0, NULL);
    context = ctx; // Copy assignment operator is defined for context
    queue = q; // Copy assignment operator is defined for queue
    cl::Program::Sources sources;
    std::string kernelcode = readFile(kerneldir);
    sources.push_back({kernelcode.c_str(), kernelcode.length()+1});

    // Create OpenCL program to link source code to context
    cl::Program program(context, sources);

    // Compile OpenCL code in execution time
    if (program.build({device}) != CL_SUCCESS) {
        std::cout << "Error building program " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;
        exit(1);
    } else {
        std::cout << "Program compiled succesfully!" << std::endl;
    }
    cl::Kernel copy_kernel(program, "copy", &error);
    assert(error == CL_SUCCESS);
    krnl_ = copy_kernel;

}

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
