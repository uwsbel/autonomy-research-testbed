#!/bin/bash

# This script setups vehicle computers

if [[ -n $SUDO_USER ]]; then
    echo "This script should not be run as root"
    exit 1
fi

echo "WARNING: You may be asked for the sudo password."
sudo -v

# =====
# Setup

mkdir -p ~/sbel

touch ~/.bashrc
echo "" >> ~/.bashrc
echo "# Custom config for sbel vehicles. DON'T CHANGE!!!" >> ~/.bashrc

# ============
# Jetson modes

if [ -f /etc/nv_tegra_release ]; then
  echo "Setting up Tegra device."

  sudo /usr/sbin/nvpmodel -m 0 # Max power

  # Set fan power mode to cool
  sudo sed -i '/FAN_DEFAULT_PROFILE/s/quiet/cool/' /etc/nvfancontrol.conf
  sudo systemctl stop nvfancontrol
  sudo rm /var/lib/nvfancontrol/status
  sudo systemctl start nvfancontrol
fi

# ======
# Apt

echo "Updating apt."

sudo apt update
sudo apt install -y tmux
sudo apt autoremove

# ======
# Docker

echo "Setting up Docker."

# Docker compose
sudo apt-get install ca-certificates curl gnupg lsb-release -y
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-compose-plugin -y

# Add nvidia as the default runtime
if [ -f /etc/nv_tegra_release ]; then
  grep -q '"default-runtime": "nvidia"' /etc/docker/daemon.json || sudo sed -i '/"data-root"/a \    "default-runtime": "nvidia",' /etc/docker/daemon.json
fi

# Create the docker group
sudo groupadd docker 2>/dev/null
# Add the user to the docker group
sudo usermod -aG docker $USER

# =========
# Miniconda

echo "Installing up miniconda."

# Install
mkdir -p ~/.conda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O ~/.conda/miniconda.sh
bash ~/.conda/miniconda.sh -b -u -p ~/.conda
rm -rf ~/.conda/miniconda.sh
~/.conda/bin/conda init bash

# Create the conda environment
~/.conda/bin/conda create -p ~/.conda/envs/sbel python=3.12 --yes

# ==============
# Clone the repo

echo "Cloning autonomy-research-testbed."

git clone https://github.com/uwsbel/autonomy-research-testbed.git ~/sbel/autonomy-research-testbed --recursive

echo "source ~/sbel/autonomy-research-testbed/vehicles/bashrc" >> ~/.bashrc

~/.conda/bin/conda run --live-stream -p ~/.conda/envs/sbel pip install -r ~/sbel/autonomy-research-testbed/requirements.txt
(cd ~/sbel/autonomy-research-testbed && ~/.conda/bin/conda run --live-stream -p ~/.conda/envs/sbel pre-commit install)
(cd ~/sbel/autonomy-research-testbed && git remote set-url origin git@github.com:uwsbel/autonomy-research-testbed.git)

echo "Done! You should logout and log back in to activate changes."
