# eplansys on ROS 2 Humble, with the two-site survey as its default command.
#
#   docker build -t eplansys .
#   docker run --rm eplansys
#   docker run --rm eplansys ros2 launch eplansys_demo survey_sites_launch.py north:=clean parallel:=true
#
# The planner (del-planner) and the grounder (plank) are built from the commits
# dependency_repos.repos pins, so the image plans exactly as the release does.

FROM ros:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# libfl-dev is popf's and libboost-filesystem-dev is plank's; neither project
# has a rosdep key that would bring them in.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libfl-dev libboost-filesystem-dev python3-vcstool ros-humble-behaviortree-cpp \
    && rm -rf /var/lib/apt/lists/*

# behaviortree_cpp 4.9.1 installs its library under lib/x86_64-linux-gnu while
# its exported CMake check looks in lib/ alone; see epistemic-humble.yaml.
RUN if [ -f /opt/ros/humble/lib/x86_64-linux-gnu/libbehaviortree_cpp.so ] && \
       [ ! -e /opt/ros/humble/lib/libbehaviortree_cpp.so ]; then \
      ln -s x86_64-linux-gnu/libbehaviortree_cpp.so /opt/ros/humble/lib/libbehaviortree_cpp.so; \
    fi

WORKDIR /eplansys_ws

# Dependencies first, so that a change to eplansys does not fetch them again.
COPY dependency_repos.repos /tmp/dependency_repos.repos
RUN mkdir -p src && vcs import src < /tmp/dependency_repos.repos

COPY . src/eplansys

RUN apt-get update && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys plank \
    && rm -rf /var/lib/apt/lists/*

# plank is named because nothing declares a build dependency on it: the
# grounder runs it as a subprocess and finds it on PATH.
RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-up-to eplansys_demo plank \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "eplansys_demo", "survey_sites_launch.py"]
