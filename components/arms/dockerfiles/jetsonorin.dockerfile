FROM public.ecr.aws/unlimited-robotics/raya.core.base_images.ubuntu.22.04_ros.humble:jetsonorin.4.37

ENV DEBIAN_FRONTEND=noninteractive

ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

ENV LOAD_SHARED_PACKAGES=true

RUN sed -i 's/PRELOAD_SHARED_LIBRARIES_DELAY=30/PRELOAD_SHARED_LIBRARIES_DELAY=1/' /setup_env.bash
RUN sed -i '/^deb https:\/\/isaac\.download\.nvidia\.com\/isaac-ros\/ubuntu\/main focal main/s/^/#/' /etc/apt/sources.list

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-moveit-msgs \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean


RUN python3 -m pip install \
    python-can==4.2.2

