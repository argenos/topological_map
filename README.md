# Topological Map

The `topological_map_ros` package uses [Networkx](https://networkx.github.io/documentation/stable/) to create a topological map representation.


## Services

* `~topological_path_plan` (topological_map_ros/TopologicalPath)  
    Get a list of nodes and directions to get from a source node to a goal node

* `~topological_position`(topological_map_ros/TopologicalPosition)  
    Get the topological position based on the pose of the robot


## Testing

### Topological Path Plan

You can do so with the following instructions:
1. Launch the topological_server: `roslaunch topological_map_ros topological_map.launch`
2. Make a service call from the terminal, e.g. `rosservice call /topological_path_plan "source: 'kitchen' goal: 'bar'"`

### Topological robot pose

When using the real robot (or at least the bringup), it's possible to test the `TopologicalPosition` service with the following instructions:

1. Launch the topological_server: `roslaunch topological_map_ros topological_map.launch`
2. Run the map server. In `mdr_environments`, there is a handy launch file if you are testing this on your laptop: `roslaunch mdr_environments rviz.launch`
3. Launch the `roslaunch topological_map_ros robot_pose_test.launch`
4. Move the robot to the new area where you want to test or, if you are using your laptop, use the `2D Pose Estimate` button in rviz to move the robot to a different place.
5. Publish from the terminal `rostopic pub /robot_pose_test/event_in std_msgs/String "data: ''"`
