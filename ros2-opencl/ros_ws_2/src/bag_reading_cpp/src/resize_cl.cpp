#include <bag_reading_cpp/image_proc.hpp>
#include "tracetools_image_pipeline/tracetools.h"
#include <chrono>

namespace image_proc
{

CLImageProc::CLImageProc(const rclcpp::NodeOptions &options)
    : Node("image_reader_node", options)
{

    initializeOpenCL();
    

    // Create a subscriber for the image topic
    // Create image pub
    pub_image_ = image_transport::create_camera_publisher(this, "resize");
    // Create image sub
    sub_image_ = image_transport::create_camera_subscription(
        this, "image",
        std::bind(
        &CLImageProc::imageCb, this,
        std::placeholders::_1,
        std::placeholders::_2), "raw");

}

CLImageProc::~CLImageProc()
{
    
}

size_t CLImageProc::get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg){
  //Serialize the Image and CameraInfo messages
  rclcpp::SerializedMessage serialized_data_img;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
  const void* image_ptr = reinterpret_cast<const void*>(image_msg.get());
  image_serialization.serialize_message(image_ptr, &serialized_data_img);
  size_t image_msg_size = serialized_data_img.size();
  return image_msg_size;
}

size_t CLImageProc::get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg){
  rclcpp::SerializedMessage serialized_data_info;
  rclcpp::Serialization<sensor_msgs::msg::CameraInfo> info_serialization;
  const void* info_ptr = reinterpret_cast<const void*>(info_msg.get());
  info_serialization.serialize_message(info_ptr, &serialized_data_info);
  size_t info_msg_size = serialized_data_info.size();
  return info_msg_size;
}

void CLImageProc::imageCb(sensor_msgs::msg::Image::ConstSharedPtr image_msg,
                        sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
    TRACEPOINT(
    image_proc_resize_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec,
    get_msg_size(image_msg),
    get_msg_size(info_msg));

    if (pub_image_.getNumSubscribers() < 1) {
    TRACEPOINT(
      image_proc_resize_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec,
      get_msg_size(image_msg),
      get_msg_size(info_msg));
    return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  } catch (cv_bridge::Exception & e) {
    TRACEPOINT(
      image_proc_resize_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec,
      get_msg_size(image_msg),
      get_msg_size(info_msg));
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    TRACEPOINT(
      image_proc_resize_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec,
      get_msg_size(image_msg),
      get_msg_size(info_msg));
    return;
  }

  TRACEPOINT(
    image_proc_resize_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);
  
    int height = height_ == -1 ? image_msg->height : height_;
    int width = width_ == -1 ? image_msg->width : width_;
    int resize_width = scale_width_ * width;
    int resize_height = scale_height_ * height;
    // cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    // std::cout << "w, h, c " << image_msg->width << " " << image_msg->height << " " << cv_ptr->image.channels() << std::endl;
    size_t buffer_size = sizeof(cl_uchar) * width * height * cv_ptr->image.channels();
    size_t resize_buffer_size = sizeof(cl_uchar) * resize_width * resize_height * cv_ptr->image.channels();

    // Create host side buffers with memory allocated on the host
    // Data will be copied over to pinned host ptrs that the host side buffer is mapped to
    cl::Buffer host_input_cl(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
                             buffer_size, NULL, &error);
    // std::cout << "Buffer create error : " << error << std::endl;
    assert(error == CL_SUCCESS);
    // cl::Buffer host_temp_cl(context, CL_MEM_READ_WRITE,
    //                         buffer_size, NULL, &error);
    // assert(error == CL_SUCCESS);
    cl::Buffer host_output_cl(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY,
                              resize_buffer_size, NULL, &error);
    assert(error == CL_SUCCESS);

    // cl::Buffer img_matrix(context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float) * 30, image_matrices, &error);
    // assert(error == CL_SUCCESS);

    // Set kernel arguments
    // error = krnl_1.setArg(0, host_input_cl);
    // assert(error == CL_SUCCESS);
    // error = krnl_1.setArg(1, host_temp_cl);
    // assert(error == CL_SUCCESS);
    // error = krnl_1.setArg(2, img_matrix);
    // assert(error == CL_SUCCESS);
    error = krnl_2.setArg(0, host_input_cl);
    assert(error == CL_SUCCESS);
    error = krnl_2.setArg(1, host_output_cl);
    assert(error == CL_SUCCESS);
    error = krnl_2.setArg(2, width);
    assert(error == CL_SUCCESS);
    error = krnl_2.setArg(3, scale);
    assert(error == CL_SUCCESS);

    // // Synchronization events
    cl::Event input_event, first_map, output_event, krnl1_event, krnl2_event;
    // Map host input buffer to pinned memory on the host
    cl_uchar *pinned_host_input = (cl_uchar *)r_queue.enqueueMapBuffer(host_input_cl, CL_FALSE, CL_MAP_WRITE_INVALIDATE_REGION, 0,
                                                                       buffer_size, NULL, NULL, &error);
    // first_map.wait();
    // Populate pinned host input
    std::memcpy(pinned_host_input, cv_ptr->image.data, buffer_size);

    // Update mapped buffer with image data
    error = w_queue.enqueueUnmapMemObject(host_input_cl, pinned_host_input, NULL, NULL);
    // input_event.wait();
    assert(pinned_host_input != NULL && error == CL_SUCCESS);
    // std::cout << int(cv_ptr->image.data[200]) << " : " << int(pinned_host_input[200]) << std::endl;

    // Enqueue kernels
    // cl::NDRange global(width, height);
    // error = k_queue.enqueueNDRangeKernel(krnl_1, 0, global, cl::NullRange, NULL, &krnl1_event);
    // // krnl1_event.wait();
    // assert(error == CL_SUCCESS);
    cl::NDRange resize_global(resize_width, resize_height, 4);
    error = k_queue.enqueueNDRangeKernel(krnl_2, 0, resize_global, cl::NullRange, NULL, &krnl2_event);
    // krnl2_event.wait();
    assert(error == CL_SUCCESS);
    k_queue.finish();
    k_queue.flush();

    // Read buffer
    cl_uchar *pinned_host_output = (cl_uchar *)r_queue.enqueueMapBuffer(host_output_cl, CL_FALSE, CL_MAP_READ, 0,
                                                                        resize_buffer_size, NULL, NULL, &error);
    error = w_queue.enqueueUnmapMemObject(host_output_cl, pinned_host_output, NULL, NULL);
    // output_event.wait();
    assert(pinned_host_output != NULL && error == CL_SUCCESS);

    // std::cout << "Read buffer error : " << error << std::endl;
    assert(error == CL_SUCCESS);
    // std::cout << "Read buffer : \n";
    cv::Mat output_image(resize_height, resize_width, CV_MAKETYPE(CV_8U, cv_ptr->image.channels()), pinned_host_output);
    cv_ptr->image = output_image;
    std::cout << "output size : " << cv_ptr->image.size() << std::endl;
  
  TRACEPOINT(
    image_proc_resize_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);

  sensor_msgs::msg::CameraInfo::SharedPtr dst_info_msg =
    std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);

  double scale_y;
  double scale_x;

  if (use_scale_) {
    scale_y = scale_height_;
    scale_x = scale_width_;
    dst_info_msg->height = static_cast<int>(info_msg->height * scale_height_);
    dst_info_msg->width = static_cast<int>(info_msg->width * scale_width_);
  } else {
    scale_y = static_cast<double>(height_) / info_msg->height;
    scale_x = static_cast<double>(width_) / info_msg->width;
    dst_info_msg->height = height_;
    dst_info_msg->width = width_;
  }

  dst_info_msg->k[0] = dst_info_msg->k[0] * scale_x;  // fx
  dst_info_msg->k[2] = dst_info_msg->k[2] * scale_x;  // cx
  dst_info_msg->k[4] = dst_info_msg->k[4] * scale_y;  // fy
  dst_info_msg->k[5] = dst_info_msg->k[5] * scale_y;  // cy

  dst_info_msg->p[0] = dst_info_msg->p[0] * scale_x;  // fx
  dst_info_msg->p[2] = dst_info_msg->p[2] * scale_x;  // cx
  dst_info_msg->p[3] = dst_info_msg->p[3] * scale_x;  // T
  dst_info_msg->p[5] = dst_info_msg->p[5] * scale_y;  // fy
  dst_info_msg->p[6] = dst_info_msg->p[6] * scale_y;  // cy

  pub_image_.publish(*cv_ptr->toImageMsg(), *dst_info_msg);

  TRACEPOINT(
    image_proc_resize_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*cv_ptr->toImageMsg())),
    static_cast<const void *>(&(*dst_info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec,
    get_msg_size(cv_ptr->toImageMsg()),
    get_msg_size(dst_info_msg));
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
    sources.push_back({resize_kernelcode.c_str(), resize_kernelcode.length() + 1});

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
    // cl::Kernel k1(program, "rectifyImageBuffer", &error);
    // assert(error == CL_SUCCESS);
    // krnl_1 = k1;
    cl::Kernel k2(program, "resizeImageBuffer", &error);
    assert(error == CL_SUCCESS);
    krnl_2 = k2;
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::CLImageProc)

