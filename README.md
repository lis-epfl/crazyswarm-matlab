# crazyswarm-matlab
High-level Matlab API for the [crazyswarm project](https://github.com/USC-ACTLab/crazyswarm). Wherever possible the naming of classes and methods follows the crazyswarm Python API.

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
Depending on your Matlab version this command will display two additional steps you need to take before the custom messages are available in Matlab.

# Requirements

- Matlab R2019a or later
- Working installation of [crazyswarm](https://github.com/USC-ACTLab/crazyswarm)