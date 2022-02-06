
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
docker run hello-world # test without sudo

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
conda install numpy scipy matplotlib ipython jupyter pandas sympy nose
conda install -c conda-forge jupyterlab
conda install scikit-learn seaborn requests pytest
conda install networkx pyparsing tqdm joblib statsmodels

# Network tools (netstat)
sudo apt install net-tools

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
# Modify the snap Exec to include argument
sudo vi /var/lib/snapd/desktop/applications/spotify_spotify.desktop
Exec=env BAMF_DESKTOP_FILE_HINT=/var/lib/snapd/desktop/applications/spotify_spotify.desktop /snap/bin/spotify --force-device-scale-factor=2.5 %U


