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
