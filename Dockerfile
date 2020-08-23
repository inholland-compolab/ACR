FROM osrf/ros:melodic-desktop-full
COPY ros/src/ /opt/tmp/src

# Set up dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    ros-melodic-openslam-gmapping\
    ros-melodic-gmapping \
    ros-melodic-rosserial \
    ros-melodic-teleop-twist-keyboard \
    libusb-1.0-0-dev \
    python-pip \
    python-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install and configure Leddar dependencies
RUN cd /opt \
    && git clone https://github.com/leddartech/LeddarSDK.git \
    && cd LeddarSDK/src \
    && chmod +x build.sh \
    && ./build.sh \
    && cd LeddarPy \
    && python setup.py install --user \
    && cd ../Py_leddar_utils \
    && python setup.py install --user

# Set up ros environment
RUN cd /opt/tmp \
    && . /opt/ros/melodic/setup.sh \
    && catkin_make \ 
    && echo ". /opt/ros/melodic/setup.sh" >> /etc/bash.bashrc \
    && echo ". /opt/tmp/devel/setup.sh" >> /etc/bash.bashrc

# NVIDIA compatibility
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


