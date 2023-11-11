# marsha_examples
This repository includes a simulation example that compares the performance of [MARSHA](https://github.com/JRL-CARI-CNR-UNIBS/marsha.git) with that of SSM and [MARS](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE.git) in a collaborative robotic cell.
The example involve a mobile humanoid mannequin sharing workspace with a robot. As the robot moves from its initial to target configuration, the human's presence in the cell prompts a response from the safety system. The compared algorithms are:

- MARSHA: Dynamically plans a safety-aware path to optimize execution time.
- SSM: Simply reduces the robot's speed.
- MARS: Replans a collision-free path, aiming for the shortest one, without accounting for safety-related implications.

## Build and execute
Therre are two options: catkin workspace or Docker.

### Catkin workspace
The software can be installed using rosinstall files.

1. Install ROS: follow the steps described in http://wiki.ros.org/ROS/Installation
2. Install Git: follow the steps described in https://git-scm.com/book/en/v2/Getting-Started-Installing-Git
3. Install wstool: follow the steps described in http://wiki.ros.org/wstool
4. Install rosdep: follow the steps described in http://wiki.ros.org/rosdep
5. Install catkin_tools: follow the steps described in https://catkin-tools.readthedocs.io/en/latest/installing.html

Create your workspace:
```
mkdir -p ~/replanning_ws/src
cd ~/replanning_ws
catkin init
wstool init src
```
The main dependency of MARSHA is [OpenMORE](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE.git). You can use its rosinstall file to dowload the required dependencies:
```
cd ~/replanning_ws
wget https://raw.githubusercontent.com/JRL-CARI-CNR-UNIBS/OpenMORE/master/OpenMORE.rosinstall
wstool merge -t src ./OpenMORE.rosinstall
wstool update -t src
```
Download the other dependencies for MARSHA:
```
cd ~/replanning_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/marsha.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/marsha_examples.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/human_aware_cost_functions.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/walking_human_simulator.git
git clone --recurse-submodules https://github.com/JRL-CARI-CNR-UNIBS/thread-pool.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/ur_simplified.git
git clone https://github.com/CNR-STIIMA-IRAS/ssm_safety.git
```
Now, compile the workspace:
```
cd ~/replanning_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build -cs
```
And source the workspace:
```
echo "source /home/$USER/replanning_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

To run the simulation, first launch the environment:
```
roslaunch marsha_tests cell.launch
```

Launch the SSM node:
```
roslaunch marsha_tests ssm.launch
```

Launch the mannequin node:
```
roslaunch marsha_tests human_simulator.launch
```

And, finally, the simulation:
```
roslaunch marsha_tests simulation.launch
```

### Docker
A [docker file](https://github.com/JRL-CARI-CNR-UNIBS/marsha_examples/blob/master/dockerfile_marsha_examples) is also available. Open a terminal, move into the folder where you have saved the docker file and run the following command:
```
sudo docker build -f dockerfile_marsha_examples -t marsha_examples .
```
Once completed, run the container:
```
xhost + 

sudo docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    marsha_examples
```
Then, inside the container:
```
roslaunch marsha_tests everything.launch
```
