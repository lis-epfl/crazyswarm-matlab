# crazyswarm-matlab
High-level Matlab API for the [_crazyswarm project_](https://github.com/USC-ACTLab/crazyswarm).
Wherever possible the naming of classes and methods follows the crazyswarm Python API.

## Download

Clone the toolbox to a folder of your choice:
```bash
git clone --recursive https://github.com/lis-epfl/crazyswarm-matlab
```

## Installation

Open Matlab and follow the steps below.

Add the toolbox to the Matlab path:
```Matlab
addpath(genpath("."))
```

Generate ROS custom messages in Matlab:
```Matlab
rosgenmsg("custom_msgs")
```
Depending on your Matlab version this command will display two additional steps you need to
take before the custom messages are available in Matlab.

## Usage

The _crazyswarm-matlab_ toolbox only works in conjunction with the `crazyswarm_server` ROS node 
of the _crazyswarm_ project. This node is responsible for the communication with the crazyflies.
Visit the [_crazyswarm_ documentation](https://crazyswarm.readthedocs.io/en/latest/usage.html#basic-flight) 
for more details and an instruction on how to launch this node. 

The _crazyswarm_ project stores all the information needed to initialize a swarm in a `crazyflies.yaml`
file. The same approach is used in _crazyswarm-matlab_ to create a `Crazyswarm` object. To ensure that 
both _crazyswarm_ and _crazyswarm-matlab_ use the same configuration file, we suggest creating a symbolic
link from the _crazyswarm-matlab_ directory to the `crazyflies.yaml` in the _crazyswarm_ directory.
Use the following command from the directory of _crazyswarm-matlab_ to do so:
``` 
ln -s PATH_TO_CRAZYSWARM/ros_ws/src/crazyswarm/launch/crazyflies.yaml .
```

# Requirements

- Matlab R2019a or later
- Working installation of [crazyswarm](https://github.com/USC-ACTLab/crazyswarm)