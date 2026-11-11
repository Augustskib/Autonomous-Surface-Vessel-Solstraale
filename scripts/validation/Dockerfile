FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions python3-rosdep python3-venv python3-pip nodejs build-essential && rm -rf /var/lib/apt/lists/*
COPY src /ws/src
WORKDIR /ws
RUN source /opt/ros/jazzy/setup.bash && rosdep update --rosdistro jazzy && apt-get update && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y && rm -rf /var/lib/apt/lists/*
COPY scripts/requirements-dataset.txt /tmp/requirements-dataset.txt
RUN python3 -m venv --system-site-packages /opt/asv-tests && /opt/asv-tests/bin/pip install -r /tmp/requirements-dataset.txt pytest
COPY . /ws
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DASV_BUILD_QUANERGY=OFF
ENV ASV_VALIDATION_CONTAINER=1
CMD ["bash", "scripts/validation/ros-checks.sh"]
