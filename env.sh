set -e

sudo apt install -y libarmadillo-dev \
                    libbullet-dev \
                    libsdl1.2-dev \
                    libsdl-image1.2-dev \
                    qtbase5-dev \
                    qtdeclarative5-dev \
                    ros-noetic-move-base-msgs \
                    ros-noetic-tf2-sensor-msgs \
# the below might only be on LattePanda (#164)
                    libyaml-cpp-dev \
                    ros-noetic-diagnostics \
                    ros-noetic-angles \
                    ros-noetic-laser-geometry \
                    ros-noetic-map-msgs \
                    ros-noetic-control-toolbox