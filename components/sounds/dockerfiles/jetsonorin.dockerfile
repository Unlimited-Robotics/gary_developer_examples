FROM public.ecr.aws/unlimited-robotics/raya.core.base_images.ubuntu.20.04_l4t.35.3.1_ros.humble:jetsonorin.4.37

ENV DEBIAN_FRONTEND=noninteractive

ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

ENV LOAD_SHARED_PACKAGES=true

RUN sed -i 's/PRELOAD_SHARED_LIBRARIES_DELAY=30/PRELOAD_SHARED_LIBRARIES_DELAY=1/' /setup_env.bash
RUN sed -i '/^deb https:\/\/isaac\.download\.nvidia\.com\/isaac-ros\/ubuntu\/main focal main/s/^/#/' /etc/apt/sources.list