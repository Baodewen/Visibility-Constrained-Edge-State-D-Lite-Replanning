# ROS1 integration skeleton

The core package has no ROS dependency. For ROS1, import `MazeNavigator`
inside a `rospy` node and connect it to your perception and motion modules.

Recommended ROS topic design:

```text
/maze/current_cell      std_msgs/Int32
/maze/goal_cell         std_msgs/Int32 or custom mission msg
/maze/edge_observation  custom msg: edge_id + state
/maze/next_cell_cmd     std_msgs/Int32
/maze/path              nav_msgs/Path or custom cell path
/maze/status            diagnostic/status string
```

For a real robot, use the planner at low frequency or event-driven timing:
run it when a cell is reached, a scan result arrives, or a new target is issued.
