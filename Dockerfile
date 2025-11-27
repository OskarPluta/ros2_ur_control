FROM osrf/ros:humble-desktop

# Ustawienie zmiennych środowiskowych, żeby Python nie buforował wyjścia
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-usb-cam \
    ros-humble-trajectory-msgs \
    ros-humble-ur-description \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install "numpy<2" opencv-python opencv-contrib-python
WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"



COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

VOLUME ["data"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ur_control_pkg", "system.launch.py"]
