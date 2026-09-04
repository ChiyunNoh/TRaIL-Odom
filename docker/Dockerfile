# Use official Ubuntu 22.04 base image
FROM ubuntu:22.04

# Install Zenoh RMW by default. It is selected at runtime by run.sh.
ARG EXPERIMENTAL_ZENOH_RMW=TRUE

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive
ENV CORE 8
ENV CERES_VERSION="2.2.0"
ENV PCL_VERSION="1.13.0"

# Env setup
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    git \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update
    
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    wget \ 
    software-properties-common \ 
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libpython3-dev \
    python3-tk \
    build-essential  \
    gedit  \
    unzip  \
    vim \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    #check if Zenoh should be installed
    $(if [ "$EXPERIMENTAL_ZENOH_RMW" = "TRUE" ]; then echo "ros-humble-rmw-zenoh-cpp"; fi) \
    && rm -rf /var/lib/apt/lists/*
    
WORKDIR /tmp
# ======== Installing Cmake library ========
RUN wget https://cmake.org/files/v3.26/cmake-3.26.3.tar.gz && \
    tar -xzvf cmake-3.26.3.tar.gz && \
    cd ./cmake-3.26.3 && \
    ./bootstrap -- -DCMAKE_BUILD_TYPE:STRING=Release && \
    make -j${CORE} && \
    make install

ARG EIGEN_VERSION=3.4.0
WORKDIR /tmp
RUN apt-get update && apt-get install -y --no-install-recommends wget ca-certificates \
 && wget -q https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz \
 && tar -xf eigen-${EIGEN_VERSION}.tar.gz \
 && cd eigen-${EIGEN_VERSION} \
 && mkdir build && cd build \
 && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \
 && make install \
 && cd /tmp && rm -rf eigen-${EIGEN_VERSION}*

WORKDIR /tmp
# ======== Installing Sophus library ========
RUN git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    mkdir build && cd build && \
    cmake -SOPHUS_USE_BASIC_LOGGING=ON .. && \
    make -j${CORE} install && \
    cd ../..

RUN git clone https://github.com/Neargye/magic_enum.git && \
    cd magic_enum && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${CORE} install && \
    cd ../..

RUN apt-get install -y libepoxy-dev

RUN git clone --branch v0.9 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin
WORKDIR /tmp/Pangolin
RUN chmod +x ./scripts/install_prerequisites.sh && \
    yes | ./scripts/install_prerequisites.sh recommended

WORKDIR /tmp/Pangolin
RUN cmake -B build \
          -DBUILD_TESTS=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build -j${CORE} && \
    cmake --install build && \
    ldconfig 

# ======== Installing PCL library ========

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list && \
    find /etc/apt/sources.list.d -type f -name "*.list" -exec sed -i 's|https://packages.ros.org/ros2/ubuntu|http://packages.ros.org/ros2/ubuntu|g' {} \;

# PCL dependencies
RUN apt-get install -y \
    libflann-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libboost-all-dev \
    libusb-1.0-0-dev \
    libusb-dev \
    libopenni-dev \
    libopenni2-dev \
    libpcap-dev \
    libpng-dev \
    mpi-default-dev \
    openmpi-bin \
    openmpi-common \
    libqhull-dev \
    libgtest-dev \
    manpages-dev

RUN apt-get update 
RUN apt-get install -y --no-install-recommends\
        ros-humble-pcl-conversions 
RUN apt-get install -y --no-install-recommends\
        ros-humble-pcl-ros\
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-${PCL_VERSION}.tar.gz 
RUN tar -xf pcl-${PCL_VERSION}.tar.gz \
    && cd pcl-pcl-${PCL_VERSION} \
    && mkdir build \
    && cd build \
    && cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \         
      -DBUILD_visualization=ON \
      -DWITH_VTK=ON \
      -DPCL_BUILD_TESTS=OFF -DPCL_BUILD_EXAMPLES=OFF \
    && make -j2\
    && make install && \
    ldconfig

RUN apt-get update && apt-get install -y pcl-tools

RUN unset PCL_VERSION


# ======== Installing Ceres library ========

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
        libunwind-dev\
        libgoogle-glog-dev libgflags-dev \
        libsuitesparse-dev libmetis-dev \
        cmake ninja-build build-essential && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${CORE} install && \
    cd ../.. && \
    rm -rf ./ceres-solver


RUN apt-get install -y libyaml-cpp-dev
RUN apt-get update && apt-get install -y libcereal-dev


WORKDIR /root/ros2_ws
RUN apt-get update 
RUN mkdir -p /tmp/jdk && cd /tmp/jdk && \   
    wget https://security.ubuntu.com/ubuntu/pool/main/o/openjdk-lts/openjdk-11-jdk-headless_11.0.27%2B6~us1-0ubuntu1~22.04_amd64.deb && \
    dpkg -i openjdk-11-jdk-headless_11.0.27+6~us1-0ubuntu1~22.04_amd64.deb || true && \
    apt-get -f install -y --no-install-recommends && \
    cd / && rm -rf /tmp/jdk && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y ros-humble-ament-cmake


WORKDIR /root/ros2_ws

RUN rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# nvidia-container-runtime
ARG NVIDIA_VISIBLE_DEVICES=all
ARG NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility

ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES}

ENTRYPOINT [ "/bin/bash" ]
