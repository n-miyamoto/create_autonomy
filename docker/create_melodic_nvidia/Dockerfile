# FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as nvidia
FROM create_ros_melodic_gazebo9
LABEL maintainer="Emiliano Borghi"

ENV USER="create"

# Remove sudo message when openning a new terminal
# https://askubuntu.com/a/22614
RUN touch ~/.sudo_as_admin_successful

USER root
# Orbbec Astra
COPY 56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules
# ASUS Xtion
RUN echo "KERNEL == \"ttyUSB0\", MODE = \"0777\"" >> /etc/udev/rules.d/80-persistent-local-usb.rules
RUN service udev reload
RUN service udev restart

USER $USER
