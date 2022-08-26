FROM  osrf/ros:noetic-desktop

# ROS/Gazebo envs
ENV QT_X11_NO_MITSHM=1
ENV IGN_IP="127.0.0.1"

# common tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip apt-utils \
    unzip htop wget curl tmux \
    python3-catkin-tools python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/*

# User defaults given here, can be easily overriden in docker-compose.yml
ARG usr_name="turtle"
ARG usr_group="turtle"
ARG usr_uid="1000"
ARG usr_gid="1000"
RUN groupadd --gid ${usr_gid} ${usr_group} &&\
  useradd --create-home --no-log-init --shell /bin/bash --uid ${usr_gid} --gid ${usr_group} ${usr_name}
RUN adduser ${usr_name} sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
  && touch /home/${usr_name}/.sudo_as_admin_successful

# ======================================================================================================================
# RUN AS END-USER
USER ${usr_name}
ENV HOME="/home/${usr_name}"
ENV PATH="$PATH:$HOME/.local/bin"
WORKDIR ${HOME}

# prep common folders
RUN mkdir -p ${HOME}/ros_workspace/src && \
  mkdir -p ${HOME}/logs                && \
  mkdir -p ${HOME}/.ros                && \
  mkdir -p ${HOME}/.gazebo

## Install jupyter notebook
RUN pip3 install --user --upgrade numpy jupyter notebook ruamel.yaml matplotlib bqplot ipywidgets voila setuptools pyyaml
RUN jupyter nbextension enable --user --py widgetsnbextension \
  &&  jupyter serverextension enable --user voila
RUN pip3 install --user \
  transforms3d \
  ipympl \
  plotly \
  tqdm
RUN jupyter nbextension install --user --py ipympl \
  && jupyter nbextension enable --user --py ipympl

## KEEP THESE LINES LAST:
## Use these lines for quick install of pip and apt packages during experimentation:
# RUN pip3 install --user \
#  <package>
# RUN sudo apt-get update -y && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
#   <package> \
# && sudo rm -rf /var/lib/apt/lists/* \
# && sudo apt-get clean

COPY --chown=${usr_name}:${usr_group} entrypoint.bash /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]
