FROM osrf/ros@sha256:93e93e09708fa1b9c0effec0e56caffad258ddbe1e89b8e4a19fe9357242665f

RUN groupmod -g 18 dialout || groupadd -g 18 dialout
RUN usermod -aG dialout,tty,video ubuntu
RUN usermod -aG irc ubuntu

RUN apt-get update && apt-get install -y \
    socat && \
    rm -rf /var/lib/apt/lists/*

USER ubuntu

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc

SHELL [ "/bin/bash" , "-c" ]


