# F1/10 @ CTU (ROS 2 port) with dynamic obstacle avoidance

This is an implementation of dynamic obstacle avoidance to CTU's **f1tenth** codebase. This code is
based on an implementation from **f1tenth-rewrite** (at `https://github.com/CTU-F1T/f1tenth-rewrite`)

## Implementation

### Map creation

The implementation of main algorithm is written in 'follow_center_line' package. It uses the map from
'cartographer_slam' and combines it with data from LIDAR to get the information about dynamic obstacles.

The points from LIDAR are binary dilated by the factor of 6, then combined with the map and the entire
map is subsequently dilated by the factor of 5. The subsequent skeletonization for path generation uses
method 'zha' which is a fast parallel algorithm for thinning digital patherns.

### Path follow

The BFS algorithm finds the nearest point on the path from the current position of the car and DFS
follows the path to the depth 40 (map cells) and checks whether the path still exists there. This is to
avoid false branches that can occur after skeletonization. The nearest point found with BFS is then taken
as a look ahead point if DFS is succesfull. The car is stopped if DFS is not succesfull.

### Unsuccelfull methods

The following methods for skeletonization were not succesfull: 'lee', 'medial_axis'.

We also tried EDF algorithm for path generation but it did not solve the problem with several branches
therefore skeletonization was use instead of.

## Building

```bash
# Clone the directory
git clone https://github.com/CTU-F1T/pvt_2023
# Get vesc and messages submodule
git submodule update --init --recusrsive
# Colcon build
colcon build --symlink-install --mixin compile-commands --packages-ignore vesc_ackermann
```

## Running

```bash
ros2 launch auto auto.ftg.launch.py config:=tx2-auto-usa.yaml
ros2 launch cartographer_slam localization.launch.py map:=track.pbstream
cp track.pbstream /home/nvidia/ros2_ws/install/storage/share/storage/share/world_
ros2 launch follow_center_line follow_line.launch.py
```
