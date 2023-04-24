# pvt_2023

## Vehicle configuration and startup

```console
$ source f1tenth/install/setup.bash
```

Note that it is required to source the setup.bash in every tmux instance.

```console
$ ros2 launch auto teensy.launch.py config:=tx2-auto-usa.yaml
$ ros2 launch auto ust10lx.launch.py
$ ros2 launch auto drive_api.launch.py config:=tx2-auto-usa.yaml
$ ros2 launch auto vesc.launch.py config:=tx2-auto-usa.yaml
$ ros2 launch vesc_ackermann vesc_to_odom_node.launch.xml
$ ros2 launch cartographer_slam mapping.launch.py
$ ros2 run tf2_ros static_transform_publisher 0.16 0 0 0 0 0 base_footprint laser
```

## RVIZ setting
Fixed fram has to be set to base_footprint.
