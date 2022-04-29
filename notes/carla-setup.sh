
# Setting up CARLA sim (0.9.13)
# Add this to .bashrc otherwise carla sim tries to use intel graphics instead of gpu
export VK_ICD_FILENAMES="/usr/share/vulkan/icd.d/nvidia_icd.json"

# Followed Debian installation process
# Create a new conda env and use this for carla
conda create --name carla python=3.7
pip install --upgrade pip
pip install pygame numpy

# 1. Set up the Debian repository in the system:
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"

# 2. Install CARLA and check for the installation in the /opt/ folder:
sudo apt-get update # Update the Debian package index
sudo apt-get install carla-simulator # Install the latest CARLA version, or update the current installation
cd /opt/carla-simulator # Open the folder where CARLA is installed

#Extract the package:
#
#    On Linux:
#        move the package to the Import folder and run the following script to extract the contents:
#
#        cd path/to/carla/root
./ImportAssets.sh

# Install PythonAPI
# CARLA provides .whl files for different Python versions. You will need to install the .whl file.
# The .whl file is found in PythonAPI/carla/dist/. There is one file per supported Python version,
# indicated by the file name (e.g., carla-0.9.12-cp36-cp36m-manylinux_2_27_x86_64.whl indicates
# Python 3.6).
#
# It is recommended to install the CARLA client library in a virtual environment to avoid conflicts
# when working with multiple versions.
#
# To install the CARLA client library, run the following command, choosing the file appropriate to
# your desired Python version. You will need pip/pip3 version 20.3 or above. See the Before you begin
# section for how to check the version and upgrade pip/pip3:
#
# If you previously installed the client library, you should uninstall the old one before installing the new one.

pip install <wheel-file-name>.whl

# Test run
cd path/to/carla/root
./CarlaUE4.sh
