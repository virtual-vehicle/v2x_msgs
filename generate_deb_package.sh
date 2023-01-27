#!/bin/bash

# This file auto-generates a debian package.
#
# Authors:
# - Christoph Pilz (https://github.com/MrMushroom)
#
# Source https://docs.ros.org/en/galactic/How-To-Guides/Building-a-Custom-Debian-Package.html
# See also https://www.theconstructsim.com/how-to-build-a-local-debian-ros2-package/

echo "Install dependencies"
sudo apt-get update
sudo apt-get -y install dh-make python3-bloom python3-rosdep fakeroot

echo "Init rosdep"
sudo rosdep init
rosdep update --include-eol-distros

echo "build the debian from package"
echo "WARNING: be sure to be in the folder of \"package.xml\""
bloom-generate rosdebian
fakeroot debian/rules binary
