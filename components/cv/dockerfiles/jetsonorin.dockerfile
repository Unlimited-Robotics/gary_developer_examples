FROM public.ecr.aws/unlimited-robotics/raya.core.base_images.ubuntu.20.04_l4t.35.3.1_ros.humble:jetsonorin.4.37

ENV DEBIAN_FRONTEND=noninteractive

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

