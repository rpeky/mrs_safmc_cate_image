# set up ros2 environment
FROM ros:humble as base

SHELL ["/bin/bash", "-c"]

# Install shared dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
vim \
cmake \
g++ \
clang \
wget \
unzip \
libgtk2.0-dev \
pkg-config \
python3-pip \
software-properties-common \
&& rm -rf /var/lib/apt/lists/*RUN

RUN add-apt-repository universe

# source ros2
RUN source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools

# set up aruco
FROM base as aruco

RUN apt-get install -y python3-prctl libatlas-base-dev ffmpeg python3-pip
RUN apt-get install -y python3-pyqt5 python3-opengl # only if you want GUI features
RUN pip3 install numpy --upgrade
#RUN pip3 install picamera2
RUN if [ "$(uname -m)" = "aarch64" ] || [ "$(uname -m)" = "armv7l" ]; then \
        apt-get update && apt-get install -y python3-pyqt5 python3-opengl python3-libcamera && \
        pip3 install picamera2; \
    else \
        echo "Skipping picamera2 for non-Raspberry Pi architecture."; \
    fi


RUN pip3 install meson ninja jinja2 ply
RUN apt install -y libyaml-dev python3-yaml python3-ply python3-jinja2
RUN apt install -y libdw-dev libunwind-dev
RUN apt install -y libudev-dev
RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt install -y libpython3-dev pybind11-dev
RUN apt install -y libevent-dev libdrm-dev libjpeg-dev libsdl2-dev
RUN git clone https://github.com/raspberrypi/libcamera.git /opt/libcamera

WORKDIR /opt/libcamera
RUN meson setup build -Dcam=enabled -Dpycamera=enabled -Dpipelines=rpi/vc4,rpi/pisp
RUN sudo ninja -C build install

RUN apt install -y libopencv-dev
RUN git clone https://github.com/kbarni/LCCV.git /opt/LCCV
RUN mkdir /opt/LCCV/build
WORKDIR /opt/LCCV/build
RUN cmake ..
RUN sudo make install

WORKDIR /home
RUN mkdir build_opencv
WORKDIR /home/build_opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip

# Create build directory and switch into it
RUN mkdir -p /build
WORKDIR /build

# Configure
RUN cmake -DWITH_GTK=ON -DWITH_GSTREAMER=ON -DOPENCV_EXTRA_MODULES_PATH=/home/build_opencv/opencv_contrib-4.x/modules /home/build_opencv/opencv-4.x

# Build
RUN cmake --build . -j3

RUN sudo make -j2 install

RUN sudo ldconfig

RUN apt-get install --no-install-recommends -y \
    ros-humble-libcamera \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-common \
    python3-numpy \
    libboost-python-dev \
    python3-opencv

WORKDIR /root

# set up PX4 stuff / dds
FROM base as px4

# Install Micro-XRCE=DDS PX4-ROS2 bridge/middleware
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /home/Micro-XRCE-DDS-Agent 
RUN mkdir build 
WORKDIR /root/Micro-XRCE-DDS-Agent/build
RUN cmake .. 
RUN make 
RUN sudo make install 
RUN sudo ldconfig /usr/local/lib/

# build workspace
FROM base as build

# copy workspace to build dependencies
WORKDIR /root
COPY ros2_ws /root/safmc_ws
RUN source /opt/ros/humble/setup.bash && \
cd /root/safmc_ws && \
colcon build --packages-select px4_msgs
RUN source install/setup.bash
colcon build --packages-select px4_ros_com
RUN source install/setup.bash
colcon build

# runtime image
FROM base as runtime

COPY --from=build /root/safmc_ws /root/safmc_ws

COPY --from=aruco /opt/libcamera /opt/libcamera
COPY --from=aruco /opt/LCCV /opt/LCCV

COPY --from=px4 /usr/local/lib /usr/local/lib

# Add entrypoints
COPY entrypoints/aruco_entrypoint.sh /aruco_entrypoint.sh
COPY entrypoints/dds_entrypoint.sh /dds_entrypoint.sh
COPY entrypoints/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /entrypoint.sh /dds_entrypoint.sh /ros_entrypoint.sh

# Default entrypoint
ENTRYPOINT [ "/bin/bash", "-c" ]
