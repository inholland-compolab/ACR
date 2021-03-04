FROM osrf/ros:melodic-desktop-full

# Set up dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    ros-melodic-openslam-gmapping\
    ros-melodic-gmapping \
    ros-melodic-cartographer \
    ros-melodic-rosserial \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-ros-numpy \
    ros-melodic-pointcloud-to-laserscan \
    arduino \
    libusb-1.0-0-dev \
    python-pip \
    python-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Download and configure Leddar dependencies
RUN cd /tmp \
    && git clone https://github.com/leddartech/LeddarSDK.git \
    && cd LeddarSDK/src \
    && chmod +x build.sh \
    && ./build.sh \
    && cd LeddarPy \
    && python setup.py install --user \
    && cd ../Py_leddar_utils \
    && python setup.py install --user

# Copy acr project to container
COPY ros/src/ /opt/acr/src
COPY arduino_src/ /opt/acr/arduino_src

# Install arduino libraries
RUN mv /opt/acr/arduino_src/libraries/* /usr/share/arduino/libraries/

# Set up ros environment
RUN cd /opt/acr \
    && . /opt/ros/melodic/setup.sh \
    && catkin_make 

# NVIDIA compatibility
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Start
ADD entrypoint.sh /opt/acr
WORKDIR /opt/acr
ENTRYPOINT ["/opt/acr/entrypoint.sh"]

# Expose ros master to host
EXPOSE 11311

