#!/usr/bin/zsh

echo "Cleaning up old builds..."
rm -rf build log install

echo "Building the project..."
colcon build --symlink-install

echo "Sourcing the workspace..."
source install/setup.zsh