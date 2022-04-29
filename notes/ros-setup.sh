# Installing ROS 2 (Galactic) on Ubuntu Linux
# Add the ROS 2 apt repository
apt-cache policy | grep universe

# Should see something like...
# 500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
#     release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64

# If you don’t see an output line like the one above, then enable the Universe repository with these instructions.
# I didn't have to do this
# sudo apt install software-properties-common
# sudo add-apt-repository universe

# Add ROS2 apt repository to system, authorize GPG key with apt
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add Repo to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install testresources
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

# Manually download ROS2 from releases page
# https://github.com/ros2/ros2/releases
#     ros2-galactic-20210616-linux-focal-amd64.tar.bz2
# Unpack
mkdir -p ~/ros2_galactic
cd ~/ros2_galactic
tar xf ~/Downloads/ros2-galactic-20210616-linux-focal-amd64.tar.bz2

# Installing and initializing rosdep
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

# Install missing dependencies
rosdep install --from-paths ~/ros2_galactic/ros2-linux/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# Install python3 libs
sudo apt install -y libpython3-dev python3-pip

# Test install
source ~/ros2_galactic/ros2-linux/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal
source ~/ros2_galactic/ros2-linux/setup.bash
ros2 run demo_nodes_py listener

## Turtle Sim
sudo apt update
sudo apt install ros-galactic-turtlesim
sudo apt install ~nros-galactic-rqt*
