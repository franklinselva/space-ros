# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

####################################################################################################
### STAGE 1: Setup the workspace and install ROS 2 dependencies
####################################################################################################
FROM ubuntu:jammy AS setup

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive
ARG INSTALL_DIR=/opt/space-ros
WORKDIR /workspace

# Set the locale
RUN apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# This variable will then act as a single source of truth.
ENV ROSDISTRO="humble"

# Add the ROS 2 apt repository
RUN apt install -y software-properties-common \
    && add-apt-repository universe \
    && apt update && apt install -y git curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install repos file generator script requirements.
RUN apt update \
    && apt install -y python3-rosinstall-generator python3-rosdep python3-vcstool \
    python3-colcon-common-extensions cmake build-essential bison wget

# Install ROS 2 dependencies
RUN rosdep init && rosdep update

# Install IKOS
RUN sudo apt-get install --yes \
    gcc g++ cmake libgmp-dev libboost-dev libboost-filesystem-dev \
    libboost-thread-dev libboost-test-dev \
    libsqlite3-dev libtbb-dev libz-dev libedit-dev \
    python3 python3-pip python3-venv \
    llvm-14 llvm-14-dev llvm-14-tools clang-14
RUN git clone -b v3.2 --depth 1 https://github.com/NASA-SW-VnV/ikos.git
RUN cd ikos \
    && mkdir build \
    && cd build \
    && cmake \
    -DCMAKE_INSTALL_PREFIX="/opt/ikos" \
    -DCMAKE_BUILD_TYPE="Debug" \
    -DLLVM_CONFIG_EXECUTABLE="/usr/lib/llvm-14/bin/llvm-config" \
    .. \
    && make \
    && make install
ENV PATH="/opt/ikos/bin/:$PATH"
RUN rm -rf ikos

# Copy inputs and generate a repos file
COPY scripts scripts
COPY spaceros-pkgs.txt spaceros-pkgs.txt
COPY excluded-pkgs.txt excluded-pkgs.txt
COPY spaceros.repos spaceros.repos

ENV AMENT_PREFIX_PATH=$INSTALL_DIR
RUN bash scripts/generate-repos.sh \
    --outfile ros2.repos \
    --packages spaceros-pkgs.txt \
    --excluded-packages excluded-pkgs.txt \
    --rosdistro $ROSDISTRO
RUN python3 scripts/merge-repos.py ros2.repos spaceros.repos -o output.repos

# VCS clone all repositories
RUN mkdir -p src \
    && vcs import src < output.repos
RUN vcs export --exact src > output.repos


RUN rosdep install -y \
    --from-paths src --ignore-src \
    --rosdistro ${ROSDISTRO} \
    # `urdfdom_headers` is cloned from source, however rosdep can't find it.
    # It is because package.xml manifest is missing. See: https://github.com/ros/urdfdom_headers
    # Additionally, IKOS must be excluded as per: https://github.com/space-ros/docker/issues/99
    --skip-keys "$(tr '\n' ' ' < 'excluded-pkgs.txt') urdfdom_headers ikos"

# Build the workspace
RUN colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --no-warn-unused-cli --install-base ${INSTALL_DIR} --parallel-workers 1 --cmake-args -DCMAKE_CXX_FLAGS="--param ggc-min-expand=20" --continue-on-error

RUN . ${INSTALL_DIR}/setup.sh && colcon test --retest-until-pass 2 --ctest-args -LE "(ikos|xfail)" --pytest-args -m "not xfail"

# Copy the repos file to the install directory
RUN cp output.repos ${INSTALL_DIR}/space-ros.repos

# Remove the workspace
RUN rm -rf /workspace

####################################################################################################
### STAGE 2: Create the final image: Core ROS 2 packages
####################################################################################################
FROM ubuntu:jammy AS core

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Space ROS"
LABEL org.label-schema.description="Preview version of the Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/space-ros"
LABEL org.label-schema.vcs-ref=${VCS_REF}

ARG DEBIAN_FRONTEND=noninteractive
ARG INSTALL_DIR=/opt/space-ros

# Set the locale
RUN apt update && apt install -y locales sudo \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add user and group
RUN useradd -ms /bin/bash space-ros
RUN usermod -aG sudo space-ros
RUN echo "space-ros ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Copy the workspace from the setup stage
COPY --from=setup ${INSTALL_DIR} ${INSTALL_DIR}
RUN chown -R space-ros:space-ros ${INSTALL_DIR}
RUN echo ". ${INSTALL_DIR}/setup.sh" >> /home/space-ros/.bashrc

RUN apt install -y software-properties-common \
    && add-apt-repository universe \
    && apt update && apt install -y git curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install required packages
RUN apt update && apt install -y python3 python3-pip libspdlog-dev \
    && pip install packaging

# Strip cache
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN rm -rf /var/cache/apt/archives
RUN pip cache purge

USER space-ros
WORKDIR /home/space-ros

####################################################################################################
### STAGE 3: Create the final image: Dev ROS 2 packages
####################################################################################################
FROM ubuntu:jammy AS dev

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Space ROS"
LABEL org.label-schema.description="Preview version of the Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/space-ros"
LABEL org.label-schema.vcs-ref=${VCS_REF}

ARG DEBIAN_FRONTEND=noninteractive
ARG INSTALL_DIR=/opt/space-ros

# Set the locale
RUN apt update && apt install -y locales sudo \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add user and group
RUN useradd -ms /bin/bash space-ros
RUN usermod -aG sudo space-ros
RUN echo "space-ros ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Copy the workspace from the setup stage
COPY --from=setup ${INSTALL_DIR} ${INSTALL_DIR}
COPY --from=setup /opt/ikos /opt/ikos
RUN echo ". ${INSTALL_DIR}/setup.sh" >> /home/space-ros/.bashrc

RUN apt install -y software-properties-common \
    && add-apt-repository universe \
    && apt update && apt install -y git curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install required packages
RUN apt update && apt install -y python3 python3-pip libspdlog-dev ros-dev-tools \
    && pip install packaging

# Strip cache
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN rm -rf /var/cache/apt/archives
RUN pip cache purge

USER space-ros
WORKDIR /home/space-ros