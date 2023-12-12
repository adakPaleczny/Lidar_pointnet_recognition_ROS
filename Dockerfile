FROM osrf/ros:noetic-desktop-full
# FROM pointcloudlibrary/env:20.04

SHELL ["/bin/bash", "-c"]

# Setup system
ENV TZ=Europe/Warsaw
RUN sudo apt update -y && sudo apt upgrade -y && apt-get install -y sudo  &&\
     sudo apt install ros-noetic-roslaunch 

RUN useradd -ms /bin/bash DV && usermod -aG sudo DV && echo 'DV     ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER DV 
WORKDIR /home/DV/ros_ws

RUN sudo chown -R DV /home/DV 
RUN mkdir src

RUN mkdir src/patchwork src/lidar_cone_detection
COPY patchwork src/patchwork
COPY lidar_cone_detection src/lidar_cone_detection

RUN rosdep update &&\
    rosdep install --from-paths src --ignore-src -r -y

# Catkin_make
RUN source /opt/ros/noetic/setup.bash && \ 
    cd /home/DV/ros_ws && catkin_make

# # Source
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/DV/.bashrc && \
    echo "source /home/DV/ros_ws/devel/setup.bash" >> /home/DV/.bashrc && \
    source /home/DV/.bashrc 