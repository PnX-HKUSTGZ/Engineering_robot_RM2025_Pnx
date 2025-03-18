#!/bin/bash

wget http://fishros.com/install -O fishros && . fishros
wget http://fishros.com/install -O fishros && . fishros

sudo apt update

sudo apt install ros-humble-asio-cmake-module
sudo apt install ros-humble-serial-driver
sudo apt install ros-humble-backward-ros
sudo apt install net-tools

sudo apt install git
git config --global user.name Demonmasterlqx
git config --global user.email 183842220@qq.com

cd ~ && mkdir .ssh && cd .ssh
ssh-keygen -t ed25519 -C "183842220@qq.com"

echo "Host github.com
  HostName github.com
  User git
  IdentityFile ~/.ssh/github_ed25519  # 或对应私钥路径
">> config

cd ~

sudo apt install build-essential cmake git pkg-config libusb-1.0-0-dev
sudo apt install libpcl-dev
sudo apt-get install libopencv-dev python3-opencv libopencv-contrib-dev

git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install

cd ~

wget https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_V3.0.1_241128.zip
tar -zxvf MVS_STD_V3.0.1_241128.zip
cd MVS_STD_V3.0.1_241128/
sudo chmod +x MVS-3.0.1_x86_64_20241128.deb
sudo dpkg -i MVS-3.0.1_x86_64_20241128.deb

cd ~
git clone https://github.com/jbeder/yaml-cpp.git
cd ./yaml-cpp
mkdir build
cd build
cmake -DYAML_BUILD_SHARED_LIBS=on ..
make -j20
sudo make install

echo export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH >> ~/.bashrc

cd ~ && mkdir code
git clone git@github.com:PnX-HKUSTGZ/Engineering_robot_RM2025_Pnx.git
cd ~/code/Engineering_robot_RM2025_Pnx/
sudo rosdepc init
rosdepc update
rosdepc install -i --from-path src --rosdistro humble -y
