Navigation
=============
This repository contains all software packages concerning the movement of a robot from one point to another point.

## Installation

Some pre-requirements:

    sudo apt-get install ros-kinetic-move-base ros-kinetic-global-planner ros-kinetic-dwa-local-planner
                         ros-kinetic-map-server
                         


Additional Dependency (for tinynurbs): glm (if not already installed)
```
cd ~/Downloads/
git clone https://github.com/g-truc/glm.git
cd glm/
git checkout 0.9.9.0 
mkdir -p build
cd build
cmake ..
sudo make install
```

Additional Dependency: tinynurbs (if not already installed)
```
cd ~/Downloads/
git clone https://github.com/ARTI-Robots/tinynurbs.git
cd tinynurbs/
mkdir -p build
cd build
cmake ..
sudo make install
```
