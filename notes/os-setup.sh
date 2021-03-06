
# Install updates and reboot
sudo apt update
sudo apt upgrade
sudo apt dist-upgrade
sudo apt autoremove
sudo apt autoclean

# Firmware update (MUST BE PLUGGED IN)
sudo fwupdmgr get-devices
sudo fwupdmgr get-updates
sudo fwupdmgr update
sudo reboot now

# Get info about graphic card and availible drivers
ubuntu-drivers devices
sudo apt install nvidia-driver-470 # pick recommended
prime-select query

# Install vim and gvim
sudo apt install vim
sudo apt install vim-gtk

# Build tools
sudo apt-get install build-essential
sudo apt install libssl-dev

sudo apt-get install libboost-all-dev

# git
sudo apt install git

# Generate ssh key and add it (add public file to github, gitlab, etc)
ssh-keygen -t ed25519 -C "leet4th@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

# Enable flatpak support
sudo apt install flatpak
sudo apt install gnome-software-plugin-flatpak
sudo flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo

# qt
sudo apt-get install mesa-common-dev
sudo apt-get install qt5-default
sudo apt-get install libcanberra-gtk-module

# xxdiff
sudo apt-get install flex bison
git clone git@github.com:blais/xxdiff.git

# build xxdiff
cd xxdiff/src
make -f Makefile.bootstrap # Generate the 'Makefile' file.
make                       # Build xxdiff

# install xxdiff binary
sudo install xxdiff/bin/xxdiff /usr/bin/
rm -rf xxdiff

# Caffeine - helper in case laptop needs to stay up all night
sudo apt install -y caffeine

# gparted, partition tool
sudo apt install gparted

sudo apt install gnome-tweak-tool

sudo apt install -y vlc

sudo snap install code --classic

# Docker
# Uninstall old versions
sudo apt-get remove docker docker-engine docker.io containerd runc

sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Setup stable repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker engine
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

# Test docker
sudo docker run hello-world

# Docker post install setup
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
docker run hello-world # test without sudo (need to restart for new group to take effect)

# Docker Compose https://docs.docker.com/compose/install/
# Download the current stable release of Docker Compose
sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
# Apply executable permissions to the binary
sudo chmod +x /usr/local/bin/docker-compose

# Download completion script to /etc/bash_completion.d/
sudo curl \
    -L https://raw.githubusercontent.com/docker/compose/1.29.2/contrib/completion/bash/docker-compose \
    -o /etc/bash_completion.d/docker-compose

# Setting up NVIDIA container toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Test NVIDIA docker setup
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# miniconda
# Download installer script from website and run it
conda config --set auto_activate_base false
conda install numpy scipy matplotlib ipython jupyter pandas sympy nose
conda install -c conda-forge jupyterlab
conda install scikit-learn seaborn requests pytest
conda install networkx pyparsing tqdm joblib statsmodels

# Network tools (netstat)
sudo apt install net-tools
sudo apt install nmap

# Sticky note app
sudo apt install xpad

# Slack
sudo snap install slack --classic

# Bitwarden password manager
sudo snap install bitwarden

# Bluejeans - OMSCS office hours
sudo snap install bluejeans --edge

# Freecad
sudo apt-get install freecad

# Spotify
# Need to open with the following arg: --force-device-scale-factor=2.5
snap install spotify

# Manually modify the snap Exec to include argument
sudo vi /var/lib/snapd/desktop/applications/spotify_spotify.desktop
Exec=env BAMF_DESKTOP_FILE_HINT=/var/lib/snapd/desktop/applications/spotify_spotify.desktop /snap/bin/spotify --force-device-scale-factor=2.5 %U

# Raspberry Pi Imager
# Manual download here https://www.raspberrypi.com/software/

# SSH raspberry pi
# Identify my network's base address
ifconfig
# scan under mask
nmap -sP <inet address from ifconfig>/24
ssh pi@<address>

# Alternativly, raspberrypi hostname should be raspberrypi.local
ping raspberrypi.local
ssh pi@raspberrypi.local

# Setup sshkey for connecting with raspberry pi
# This will add the pub key to .ssh/authorized_keys on raspberrypi
ssh-copy-id -i ~/.ssh/id_ed25519 pi@<address>
ssh-copy-id -i ~/.ssh/id_ed25519 pi@raspberrypi.local

# HEIC file format from iphone live photos
sudo apt install heif-gdk-pixbuf

# Gimp
sudo apt install gimp


# Teensy dev
sudo apt-get install libusb-dev

# PCL
sudo apt install libpcl-dev

# GTSAM release 4.0 (2020-10-17)
# https://launchpad.net/~borglab
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev

