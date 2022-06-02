#!/bin/bash

groupmod -g 987 uucp
usermod -aG uucp art
sudo apt-get update
sudo apt-get install build-essential
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install qtcreator
sudo apt-get install qt5-default
sudo apt-get install libqt5opengl5
sudo apt install -y curl
curl -O http://ftp.de.debian.org/debian/pool/main/d/double-conversion/libdouble-conversion1_3.1.0-3_amd64.deb
sudo dpkg -i ./libdouble-conversion1_3.1.0-3_amd64.deb
rm ./libdouble-conversion1_3.1.0-3_amd64.deb
sudo apt-get install wget
wget http://security.ubuntu.com/ubuntu/pool/main/i/icu/libicu60_60.2-3ubuntu3.2_amd64.deb
sudo apt-get install ./libicu60_60.2-3ubuntu3.2_amd64.deb
rm ./libicu60_60.2-3ubuntu3.2_amd64.deb
sudo apt-get install rapidjson-dev
