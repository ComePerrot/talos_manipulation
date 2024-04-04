# Deburring ROS interface

Interface to use the Deburring Model Predictive controller inside of a ROS architecture.

This package uses the API build in [linear-feedback-controller-msgs](https://github.com/loco-3d/linear-feedback-controller-msgs) to exchange data.

## Example

After building the package and its dependencies, the controller can be launched using the following command:
```
roslaunch deburring_ros_interface talos_deburring.launch
```

This command needs to be launched in conjontion with a robot controller, the one we use for TALOS is called [linear-feedback-controller](https://github.com/loco-3d/linear-feedback-controller).

## Dependencies

* [linear-feedback-controller-msgs](https://github.com/loco-3d/linear-feedback-controller-msgs)
* [pal_statistics](https://github.com/pal-robotics/pal_statistics)
