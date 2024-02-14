FROM --platform=linux/arm64 ros:humble-ros-core-jammy
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg --add-architecture arm64

RUN apt-get update && apt-get -y upgrade \
  && apt-get install -y \
    apt-utils \
    unzip \
    tar \
    xz-utils \
    ocl-icd-* \
    opencl-headers \
    clinfo \
    python3-bt2 \
    ;

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
        python3 python3-pip git build-essential \
        ca-certificates curl gnupg lsb-release locales \
        libtinfo5 unzip kmod

RUN apt-get -y install curl build-essential libssl-dev git wget \
    ocl-icd-* opencl-headers python3-vcstool \
    python3-colcon-common-extensions python3-colcon-mixin \
    kpartx u-boot-tools pv ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds* python3-bt2


RUN mkdir -p /home/user/DEVELOPMENT/
RUN apt-get update && apt-get install -y terminator

RUN apt-get update && apt-get install -y g++-aarch64-linux-gnu  gcc-aarch64-linux-gnu python3-rosdep

SHELL ["/bin/bash", "-c"]
RUN if [ -d "/tmp/tracing_ws" ]; then rm -r /tmp/tracing_ws; fi
RUN ros2 run tracetools status || echo $(mkdir -p /tmp/tracing_ws/src && \
        cd /tmp/tracing_ws/src && \
        git clone https://github.com/ros2/ros2_tracing -b humble && \
        cd /tmp/tracing_ws && \
        colcon build --merge-install) && export FIXEDTRACING=true
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash
RUN if [ ${FIXEDTRACING} ]; then source /tmp/tracing_ws/install/setup.bash; else true; fi
# RUN ros2 run tracetools status

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
          ros-humble-theora-image-transport ros-humble-gazebo-msgs \
          ros-humble-image-transport ros-humble-tracetools-* \
          ros-humble-image-common lttng-tools liblttng-ust-dev \
          liblttng-ust-common1 babeltrace2 \
          python3-lttng python3-lttnganalyses python3-lttngust \
          python3-bt2 babeltrace2 ros-humble-rqt-graph \
          ros-humble-ros2bag* ros-humble-rosbag2* \
          ros-humble-vision-opencv  && \
        rm -rf /var/lib/apt/lists/*    

RUN mkdir -p /framework_ws/src
WORKDIR /framework_ws
COPY framework_humble.repos ./framework_humble.repos
RUN cat framework_humble.repos && \
        vcs import src --recursive < framework_humble.repos
RUN rosdep init
RUN rosdep update || true && apt-get update
RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y \
            --skip-keys "console_bridge fastcdr fastrtps libopensplice67 \
                libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers \
                ament_cuda isaac_ros_image_proc acceleration_firmware_kr260 acceleration_firmware_BOARD \
                acceleration_firmware_kv260 ocl-icd-libopencl1 ocl-icd-dev"


# RUN pip install graphviz 
# RUN apt-get update && apt-get install -y babeltrace2
# RUN /bin/bash -c "\
#         source /opt/ros/humble/setup.bash "

