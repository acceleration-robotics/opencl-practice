#include <bag_reading_cpp/2_nodes_image_proc.hpp>
#include <chrono>

using namespace std::chrono_literals;

CLImageProc::CLImageProc(const std::string &bag_file_path)
    : Node("image_reader_node"), bag_file_path_(bag_file_path)
{

    initializeOpenCL();
    rosbag2_cpp::StorageOptions storage_options;
    storage_options.uri = bag_file_path_;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // Create a subscriber for the image topic

    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { processImage(msg); });

    info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
        { getCameraInfo(msg); });

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&CLImageProc::timerCallback, this));

    // Open the bag file for reading with default options
    rosbag_reader_ = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
    rosbag_reader_->open(storage_options, converter_options); // Provide an empty ConverterOptions
}

CLImageProc::~CLImageProc()
{
    // No need to explicitly close the bag file, smart pointer handles it
    rosbag_reader_->reset();
}

void CLImageProc::processImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
    width = msg->width;
    height = msg->height;
    int scale = 1;
    resize_width = scale * width;
    resize_height = scale * height;
    cv::Mat image = cv_bridge::toCvCopy(msg, "rgba8")->image;

    auto start = std::chrono::high_resolution_clock::now();

    // Create device side buffers
    size_t buffer_size = sizeof(cl_uchar) * width * height * image.channels();
    size_t resize_buffer_size = sizeof(cl_uchar) * resize_width * resize_height * image.channels();

    // Create host side buffers with memory allocated on the host
    // Data will be copied over to pinned host ptrs that the host side buffer is mapped to
    cl::Buffer host_input_cl(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
                             buffer_size, NULL, &error);
    assert(error == CL_SUCCESS);
    // cl::Buffer host_temp_cl(context, CL_MEM_READ_WRITE,
    //                         buffer_size, NULL, &error);
    // assert(error == CL_SUCCESS);
    cl::Buffer host_output_cl(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY,
                              resize_buffer_size, NULL, &error);
    assert(error == CL_SUCCESS);

    cl::Buffer img_matrix(context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 30, image_matrices, &error);
    assert(error == CL_SUCCESS);

    // Set kernel arguments
    error = krnl_1.setArg(0, host_input_cl);
    assert(error == CL_SUCCESS);
    error = krnl_1.setArg(1, host_output_cl);
    assert(error == CL_SUCCESS);
    error = krnl_1.setArg(2, img_matrix);
    assert(error == CL_SUCCESS);
    // error = krnl_2.setArg(0, host_input_cl);
    // assert(error == CL_SUCCESS);
    // error = krnl_2.setArg(1, host_output_cl);
    // assert(error == CL_SUCCESS);
    // error = krnl_2.setArg(2, width);
    // assert(error == CL_SUCCESS);
    // error = krnl_2.setArg(3, scale);
    // assert(error == CL_SUCCESS);

    // // Synchronization events
    cl::Event input_event, first_map, output_event, krnl1_event, krnl2_event;
    // Map host input buffer to pinned memory on the host
    cl_uchar *pinned_host_input = (cl_uchar *)r_queue.enqueueMapBuffer(host_input_cl, CL_FALSE, CL_MAP_WRITE_INVALIDATE_REGION, 0,
                                                                       buffer_size, NULL, NULL, &error);
    // first_map.wait();
    // Populate pinned host input
    std::memcpy(pinned_host_input, image.data, buffer_size);

    // Update mapped buffer with image data
    error = w_queue.enqueueUnmapMemObject(host_input_cl, pinned_host_input, NULL, NULL);
    // input_event.wait();
    assert(pinned_host_input != NULL && error == CL_SUCCESS);
    std::cout << int(image.data[200]) << " : " << int(pinned_host_input[200]) << std::endl;

    // Enqueue kernels
    cl::NDRange global(width, height);
    error = k_queue.enqueueNDRangeKernel(krnl_1, 0, global, cl::NullRange, NULL, &krnl1_event);
    // krnl1_event.wait();
    assert(error == CL_SUCCESS);
    // cl::NDRange resize_global(resize_width, resize_height, 4);
    // error = k_queue.enqueueNDRangeKernel(krnl_2, 0, resize_global, cl::NullRange, NULL, &krnl2_event);
    // krnl2_event.wait();
    // assert(error == CL_SUCCESS);
    k_queue.finish();
    k_queue.flush();

    // Read buffer
    cl_uchar *pinned_host_output = (cl_uchar *)r_queue.enqueueMapBuffer(host_output_cl, CL_FALSE, CL_MAP_READ, 0,
                                                                        buffer_size, NULL, NULL, &error);
    error = w_queue.enqueueUnmapMemObject(host_output_cl, pinned_host_output, NULL, NULL);
    // output_event.wait();
    assert(pinned_host_output != NULL && error == CL_SUCCESS);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Read buffer error : " << error << std::endl;
    assert(error == CL_SUCCESS);
    std::cout << "Read buffer : \n";
    cv::Mat output_image(height, width, CV_MAKETYPE(CV_8U, image.channels()), pinned_host_output);

    std::string new_line = std::to_string(get_time(input_event, 2)) + ","  + 
                           std::to_string(get_time(output_event, 2)) + ","+ std::to_string(duration.count()) + ",\n";
    std::ofstream myfile;
    myfile.open("/root/DEVELOPMENT/opencl-test-codes/ros2-opencl/ros_ws/analyses/rectify_cl.csv", std::ios::app);
    myfile << new_line;

    // Save the image using OpenCV
    std::string filename = "/root/DEVELOPMENT/opencl-test-codes/ros2-opencl/ros_ws/saved_images/rectify_cl_image_" + std::to_string(image_count_) + ".png";
    cv::imwrite(filename, output_image);
    image_count_++;
}

void CLImageProc::getCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received Camera Info");
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

void CLImageProc::timerCallback()
{
    if (!rosbag_reader_->has_next())
    { // Check if there are any remaining messages
        RCLCPP_INFO(this->get_logger(), "No more messages in the bag file. Exiting...");
        rclcpp::shutdown(); // Exit the node
        return;
    }
    while (rosbag_reader_->has_next())
    {
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_msg = rosbag_reader_->read_next();

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

void CLImageProc::initializeOpenCL()
{
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

    cl::CommandQueue qr(context, device, CL_QUEUE_PROFILING_ENABLE, NULL); // Read command queue
    cl::CommandQueue qw(context, device, CL_QUEUE_PROFILING_ENABLE, NULL); // Write command queue
    cl::CommandQueue qk(context, device, CL_QUEUE_PROFILING_ENABLE, NULL); // Kernel enqueue command queue

    r_queue = std::move(qr);
    w_queue = std::move(qw);
    k_queue = std::move(qk);
    // context = ctx; // Copy assignment operator is defined for context
    // queue = q; // Copy assignment operator is defined for queue
    cl::Program::Sources sources;
    sources.push_back({kernelcode.c_str(), kernelcode.length() + 1});

    // Create OpenCL program to link source code to context
    cl::Program p(context, sources);
    program = std::move(p);

    // Compile OpenCL code in execution time
    if (program.build({device}) != CL_SUCCESS)
    {
        std::cout << "Error building program " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;
        exit(1);
    }
    else
    {
        std::cout << "Program compiled succesfully!" << std::endl;
    }
    cl::Kernel k1(program, "rectifyImageBuffer", &error);
    assert(error == CL_SUCCESS);
    krnl_1 = k1;
    // cl::Kernel k2(program, "resizeImageBuffer", &error);
    // assert(error == CL_SUCCESS);
    // krnl_2 = k2;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
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
