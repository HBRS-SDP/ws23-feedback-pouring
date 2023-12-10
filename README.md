# ws23-feedback-pouring
Pouring the liquid/cereals using Kinnova arm with force estimation feedback

## Setup
Create a workspace in your local machine

```bash
mkdir ~/feedback_pouring && cd ~/feedback_pouring

mkdir src
mkdir build
```

clone the repository in src 
```bash
cd ~/feedback_pouring/src
git clone https://github.com/HBRS-SDP/ws23-feedback-pouring.git .
```

Adding kdl-parser
```bash
git clone https://github.com/orocos/orocos_kinematics_dynamics.git

# clone this inside the repository

# kdl-parser
sudo apt-get install libkdl-parser-dev

#urdf
sudo apt-get install liburdfdom-dev

```

## Build
```bash
cd ~/feedback_pouring/build

# cmake
cmake ../src/

# build
cmake --build .
```

## run
```bash
cd ~/feedback_pouring/outputs/feedback_pouring

# run the executable file
./filename
```
