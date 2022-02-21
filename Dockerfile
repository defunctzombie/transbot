FROM ros:noetic-ros-core as base

RUN apt update
RUN apt install ros-noetic-rosbridge-suite -y
RUN apt install make g++ -y
RUN apt install python3-pip -y
RUN apt install ros-noetic-usb-cam -y
RUN apt install ros-noetic-rplidar-ros -y

COPY workspace/requirements.txt /opt/transbot/workspace/
RUN pip install -r /opt/transbot/workspace/requirements.txt

# disable the evils of pycache litter everywhere
ENV PYTHONDONTWRITEBYTECODE 1

# http://wiki.ros.org/ROS/EnvironmentVariables#ROS_HOME
ENV ROS_HOME /opt/transbot/var/roshome

RUN mkdir -p /opt/transbot/var/bags
RUN mkdir -p /opt/transbot/var/roshome
RUN mkdir -p /opt/transbot/etc

RUN touch /opt/transbot/etc/params.yaml
RUN touch /opt/transbot/etc/foxglove_api_key.txt

VOLUME /opt/transbot/var
VOLUME /opt/transbot/etc

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

FROM base as prod

COPY build.sh /

WORKDIR /opt/transbot/workspace
COPY ./workspace /opt/transbot/workspace

RUN "/build.sh"
CMD ["roslaunch", "launch", "transbot.launch"]