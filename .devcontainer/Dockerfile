FROM osrf/ros:jazzy-desktop-full-noble

RUN groupmod -g 18 dialout || groupadd -g 18 dialout
RUN usermod -aG dialout,tty,video ubuntu
RUN usermod -aG irc ubuntu

USER ubuntu

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc

SHELL [ "/bin/bash" , "-c" ]


