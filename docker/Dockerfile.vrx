FROM swarm-formation-qn:five-finite-wire

# Separate optional environment image; existing qn/PVS images are unchanged.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-hector-gazebo-plugins ros-noetic-robot-localization \
    ros-noetic-velodyne-gazebo-plugins ros-noetic-geographic-msgs \
    ros-noetic-joy ros-noetic-joy-teleop
COPY . /vrx_ws/src/vrx
WORKDIR /vrx_ws
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin_make -j2 -DCMAKE_BUILD_TYPE=Release'
ENV GAZEBO_MODEL_DATABASE_URI=""
WORKDIR /workspace
