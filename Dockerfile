ARG ROS_DISTRO
ARG DCMAKE_BUILD_TYPE
FROM ros:${ROS_DISTRO}

RUN apt update && \
    apt install -y libeigen3-dev
RUN mkdir ros2_dev && \
    cd ros2_dev && \
    apt update && \
    git clone https://github.com/juanheliosg/iiwa_ros2 && \
    apt-get install -y pip && \
    cd iiwa_ros2 && \
    vcs import < iiwa_ros2.repos && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --symlink-install

RUN apt install -y nano
#debugging libs
RUN apt install gdb -y && apt install xterm -y 

RUN apt install valgrind -y

RUN mkdir /ros_app

COPY requirements.txt /ros_app/requirements.txt
COPY iiwa_controllers.yaml /ros_app/iiwa_controllers.yaml
RUN yes | pip install -r /ros_app/requirements.txt

COPY robots_models /ros_app/robots_models

RUN cd /ros_app/robots_models && \
    python3 setup.py sdist bdist_wheel && \
    pip install --user /ros_app/robots_models/dist/robots_models-0.1.tar.gz

RUN apt install librdkafka-dev -y && \
    apt install pkg-config -y && \
    apt install libglib2.0-dev -y && \
    apt install psmisc

RUN apt install libfmt-dev -y

# Install library for bayesian multi-objective optimization
RUN git clone https://github.com/ppgaluzio/MOBOpt.git && \
    cd MOBOpt && \
    python3 setup.py install && \
    pip3 install https://github.com/ppgaluzio/MOBOpt/archive/master.zip

COPY src /ros_app/src
COPY other /ros_app/other

#installing kafka libraries for monitoring

RUN sed -i 's|export GAZEBO_MASTER_URI=http://localhost:11345|export GAZEBO_MASTER_URI=\${GAZEBO_MASTER_URI}|g' /usr/share/gazebo/setup.sh

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ros2_dev/iiwa_ros2/install/setup.sh && \
    . /usr/share/gazebo/setup.sh && \
    cd /ros_app && \ 
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=${DCMAKE_BUILD_TYPE} --symlink-install

RUN sed -i 's|from scipy import integrate, randn|from scipy import integrate\nfrom numpy.random import randn|g' /usr/local/lib/python3.10/dist-packages/roboticstoolbox/mobile/EKF.py

ENV BASE_DIR=/ros_app
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 755 ros_entrypoint.sh
#to rm 
RUN mkdir /ros_app/PD

COPY initial_positions.yaml /ros_app/initial_positions.yaml

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]

