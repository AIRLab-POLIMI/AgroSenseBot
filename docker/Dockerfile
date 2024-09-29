FROM osrf/ros:jazzy-desktop

#ARG user_uid
ARG user_name=asb
#ARG user_gid
ARG user_group=asb

# create and switch to asb user
RUN sudo userdel -r ubuntu
RUN useradd -ms /bin/bash $user_name
RUN adduser $user_name sudo
RUN echo "$user_name ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER $user_name:$user_group

# Install dependences
RUN sudo apt-get update && sudo apt install -y software-properties-common && sudo add-apt-repository -y ppa:lely/ppa
RUN \
    sudo apt update && \
    sudo apt install -y \
        liblely-coapp-dev liblely-co-tools python3-dcf-tools can-utils \
        openssh-server byobu \
        libqwt-qt5-dev \
        xterm \
        ros-jazzy-xacro ros-jazzy-ros2controlcli ros-jazzy-ros2-controllers-test-nodes ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster ros-jazzy-webots-ros2 ros-jazzy-rqt-gui ros-jazzy-rqt-tf-tree ros-jazzy-nav2-* ros-jazzy-robot-localization ros-jazzy-tf-transformations \
        ros-jazzy-microstrain-inertial-driver ros-jazzy-microstrain-inertial-rqt ros-jazzy-ntrip-client \
        ros-jazzy-ouster-ros \
        ros-jazzy-octomap-ros ros-jazzy-octomap-server \
        ros-jazzy-interactive-markers \
        python3-pip python3-typing-extensions python3-transforms3d python3-websocket

RUN sudo apt update && sudo apt install -y bash-completion  # TODO remove ros-jazzy-octomap and only use liboctomap?

RUN pip3 install --break-system-packages rosbags

# Note: required host dependency: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
# Install webots
#RUN sudo mkdir -p /etc/apt/keyrings && cd /etc/apt/keyrings && sudo wget -q https://cyberbotics.com/Cyberbotics.asc
#RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
#RUN sudo apt update && sudo apt install -y webots

# clone repo
RUN mkdir -p ~/asb_logs
RUN mkdir ~/w
# RUN mkdir -p ~/w/agrosensebot_ws/src/ TODO
# RUN cd ~/w/agrosensebot_ws/src/ TODO
# RUN git clone --branch=... TODO

# RUN rosdep update && cd ~/w/agrosensebot_ws && rosdep install --from-paths src --ignore-src -r -y

# link bash files
RUN mv ~/.bashrc ~/.bashrc_
RUN ln -s ~/w/agrosensebot_ws/src/AgroSenseBot/asb_etc/files/.bashrc ~/.bashrc
RUN ln -s ~/w/agrosensebot_ws/src/AgroSenseBot/asb_etc/files/.bash_aliases ~/.bash_aliases
#
## copy bashrc files
#COPY .bashrc /home/$user_name/.bashrc
#RUN sudo chown $user_name:$user_name /home/$user_name/.bashrc
#COPY .bash_aliases /home/$user_name/.bash_aliases
#RUN sudo chown $user_name:$user_name /home/$user_name/.bash_aliases
