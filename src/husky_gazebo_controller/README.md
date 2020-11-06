## Husky gazebo controller

This ros node uses a custom mobilenetv2 neural network implemented in Pytorch, which classifies four defined classes:

- bicycle
- car
- cat
- dog

>When a bicycle or a car is detected, the husky moves directly to the object and stops when  reaches a defined point in order to avoid a crash.


>Similarly, in case a cat or a dog is detected, the husky moves to the opposite point of the object coordinates.

### Requirements

In order to execute the inference on mobilenetv2, make sure to install the requirements specified in the included [requirements.txt](requirements.txt) file, using the following command:


```sh
$ pip3 install -r requirements
```

### How to run the gazebo controller

Once the gazebo environment and sound nodes are running, execute this ros node using the following command:

```sh
$ rosrun husky_gazebo_controller husky_controller.py
```

Example log output message:


>[INFO] [1604633499.773156, 3549.002000]: Detected class: bicycle
>[INFO] [1604633499.779318, 3549.004000]: Goal position x: -10.0
>[INFO] [1604633499.782077, 3549.006000]: Goal position y: 10.0

