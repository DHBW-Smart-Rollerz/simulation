# ROS2 Simulation Package

[![Build Test](https://github.com/DHBW-Smart-Rollerz/simulation/actions/workflows/build-test.yaml/badge.svg)](https://github.com/DHBW-Smart-Rollerz/simulation/actions/workflows/build-test.yaml)

This package provides a simulation environment for Smarty, built using ROS 2 and Gazebo. It allows you to define custom roads, visualize Smarty's behavior, and test various scenarios.


## Installation
Before installation, ensure you have ROS 2 installed and your [smarty workspace](https://github.com/DHBW-Smart-Rollerz/.dotfiles/wiki/5.-Install-Smarty-Software-Stack) is properly set up.

1. Clone this repository into the `src` directory of your smarty workspace: 
```bash
    cd smarty_workspace/src
    git clone <repository_url>
```
2. Install necessary libraries
```bash
    sudo apt install libcairo2-dev
    make install_python # installs pip dependecies from requirements.txt
```
3. Build the simulation package
```bash
    make build PKG=simulation # remove the 'PKG' argument to build the whole workspace
``` 
4. If the build fails, clean and rebuild the entire workspace 
```bash
    make clean
    make build
``` 
## Usage

### Launch simulation

To launch the simulation environment, execute the following command. This will start Gazebo (the simulator) along with RViz for visualization.
```bash
    ros2 launch simulation simulation.launch.py
``` 
After that you can start the rest of the smarty software stack if needed. No other commands are needed for mocking the actual car with the simulation. The simulation is a "drop-in" replacement for the actual smarty.

**Run smarty software stack:**
```bash
    ros2 launch camera_preprocessing camera_preprocessing.launch.py
    ros2 launch lane_detection_ai lane_detection_ai.launch.py debug:=true # -> if debug image is desired
    ros2 launch pathplanning pathplanning.launch.py
    ros2 run querregelung querregelung
    ros2 topic pub /control/velocity/target std_msgs/Float32 "{data: 1.0}"
``` 

If the simulation runs very slow especially when the whole software stack is running, this is probably due to your PC being not strong enough to handle all of it. In this case you must live with it. Everything should work fine and just run more slowly.
At the bottom right of the 3D scene you can see a percentage. If you can't see it - it's black text in front of the black road, so move the view a little. This number indicates the percentage of time that passes in the simulation compared to the real world. A low number is bad and you should reach 30% for the simulation to be usable.


### Create new road

1. Define new road layouts by creating models within the `models/roads` directory. You can take a look at the default road to get some ideas on how to create a new road.
2. Update the `ros_params.yaml` configuration file located in the `config/` directory to specify the newly created road model you want to use.

## Structure

- `config/`: ROS Node parameters, RViz config and Mapping config between ROS and Gazebo
- `launch/`: Launch file to start the simulation
- `models/`: Contains the World and Smarty model together with its meshes as well as the defined roads 
- `resource/`: Contains the package name (required to build with colcon)
- `simulation`: Contains the required nodes to launch and communicate with gazebo
- `test/`: Contains all tests
- `package.xml`: Contains metadata about the package
- `setup.py`: Used for Python package configuration
- `setup.cfg`: Additional configuration for the package
- `requirements.txt`: Python dependencies

## Contributing

Thank you for considering contributing to this repository! Here are a few guidelines to get you started:

1. Fork the repository and clone it locally.
2. Create a new branch for your contribution.
3. Make your changes and ensure they are properly tested.
4. Commit your changes and push them to your forked repository.
5. Submit a pull request with a clear description of your changes.

We appreciate your contributions and look forward to reviewing them!

## License

This repository is licensed under the MIT license. See [LICENSE](LICENSE) for details.
