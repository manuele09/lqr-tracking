# Configuration Files

The configuration files are saved in the `config` directory.

## NeuralNet.json


Contains information necessary to correctly import the neural network responsible for control.

```json
{
    "weights_paths": "Path to the directory containing weights files. Files must have the format: W1.csv, B1.csv, W2.csv, B2.csv, ...",
    "num_layers": "Number of layers in the neural network",
    "isQuaternion": "Boolean indicating whether the neural network uses quaternions",
    "num_output_neurons": "Number of output neurons for the quaternion network (if used). Allowed values: 1 or 2"
}
```
## TestReference.json

Contains information for testing the controller behavior on a set of reference states.

```json
{
    "testDuration": "Testing duration for each reference state in seconds (double)",
    "logInterval": "Time interval for logging simulation data in seconds (double)",
    "referenceList": "List of reference vectors to test"
}
```

# Compilation
In the workspace directory, run the following commands:
```bash
catkin clean -y  # (if necessary)
catkin build
```

# Execution

In one terminal, navigate to the PX4-autopilot directory and run the following command to start PX4 SITL in Gazebo:
```bash
make px4_sitl gazebo
```

In another terminal, navigate to the workspace and run the following command to launch MAVROS:
```bash
source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

In another terminal, navigate to the workspace and run the following command to launch the controller:
```bash
source devel/setup.bash
roslaunch lqr_controller lqr_euler.launch
```

