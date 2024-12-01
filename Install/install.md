# Installation

## Install with conda (Linux)

### Basic Env

> ***Note**: This will only install the basic module. For other algorithm submodules, please follow the instructions [Algorithm Submodule Env](#algorithm-submodule-env) to install as needed.*

1. Pull the repository and update the submodule
```
cd /home/$(whoami)
git clone https://github.com/yding25/BestMan_Flexiv.git
```

<!-- - Integrate with flexiv_rdk 
Ensure you have flexiv_rdk version 0.10. You can download it from [Flexiv Robotics GitHub](https://github.com/flexivrobotics/flexiv_rdk.git) or the [Flexiv RDK](https://rdk.flexiv.com/manual/getting_started.html#setup-and-run-python-rdk) Manual. -->

2. Run the following script to add the project to the PYTHON search path

```
/home/$(whoami)/BestMan_Flexiv/Install
chmod 777 pythonpath.sh
bash pythonpath.sh
source ~/.bashrc
```

3. For Flexiv, add the flexivrdk to the PYTHON search path

```
chmod 777 load_flexivrdk.sh
bash load_flexivrdk.sh
source ~/.bashrc
```

4. Create basic conda environment
```
cd /home/$(whoami)/BestMan_Flexiv/Install
conda env create -f basic_env_py38.yaml
```

5. Install ROS environment
```
cd /home/$(whoami)/BestMan_Flexiv/Install
chmod 777 install_ros_noetic.sh
bash install_ros_noetic.sh
source ~/.bashrc
```

6. Check ROS environment
```
roscore
echo $ROS_DISTRO
```
