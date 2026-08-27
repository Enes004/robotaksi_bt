# robotaksi_bt

A ROS 2 (Humble) behavior tree package built with [BT.CPP v3](https://www.behaviortree.dev/), developed for the **Teknofest Robotaksi Autonomous Vehicle Competition**. This package implements the decision-making layer of the vehicle: it consumes map and perception data, plans routes, and dispatches navigation goals to Nav2 while managing mission logic and safety behaviors.

## Architecture

```
+-------------------+     +--------------------+     +--------------------+     +---------+
|   Semantic Map    | --> |   Behavior Tree     | --> |   Nav2 (action)    | --> | Vehicle |
| (lanelet/GeoJSON)  |     |   (this package)    |     | navigate_to_pose   |     +---------+
+-------------------+     +--------------------+     +--------------------+
                                    ^
                                    |
                           +------------------+
                           |    Perception     |
                           |   (YOLO, ROS2)     |
                           +------------------+
```

The behavior tree is the single decision authority: it reads the current route segment from the map, checks perception state (traffic signs, lights, obstacles), and issues a single navigation goal to Nav2 per segment. Nav2 is responsible for local path execution and obstacle avoidance; it has no knowledge of the semantic map.

## Core components

| Component | Description |
|---|---|
| **Map loader & router** (`segment_graph.hpp`) | Loads a lanelet-based semantic map (GeoJSON: lanelet layer + linestring layer). Builds an undirected adjacency graph and finds routes with Dijkstra's algorithm. Direction of travel per segment is derived from route topology after planning, not from the source line direction (which reflects drawing order, not traffic flow). |
| **Segment-based behavior dispatch** | Routes are decomposed into typed segments (`LANE_FOLLOW`, `INTERSECTION`, `ROUNDABOUT`, `TUNNEL`, `PASSENGER_STOP`, `PARKING`). Each type triggers a dedicated subtree with type-specific behavior (speed limits, headlights, dwell timing). |
| **Safety reflex layer** | Reactive condition/action pairs for pedestrian, dynamic obstacle, and static obstacle detection, evaluated every tick independently of the active mission subtree. |
| **Perception bridge** (`PerceptionProvider`) | Subscribes to a single detection topic (`Detection2DArray`-based) from the perception stack and exposes typed queries (sign class, traffic light color, confidence) to multiple BT nodes without duplicate subscriptions. |
| **Nav2 client** (`FollowLaneSegment`) | Wraps `nav2_msgs::action::NavigateToPose` as a `StatefulActionNode`, translating the current segment goal into a navigation request and reporting success/failure back to the tree. |
| **Odometry provider** | Lazy-initialized singleton subscription to localization output, used by stop-accuracy and stuck-detection nodes. |

## Repository structure

```
include/robotaksi_bt/     Node headers, segment_graph.hpp, shared providers
src/                       Node implementations
behaviour_trees/           BT XML tree definitions
config/                    Map data (lanelet / linestring / vertex layers)
scripts/                   Sample mission files for local testing
```

## Build

```bash
colcon build --packages-select robotaksi_bt
source install/setup.bash
```

## Run (standalone test)

```bash
ros2 run robotaksi_bt bt_test <path_to_tree.xml> \
  --ros-args \
  -p tour:=<tour_number> \
  -p mission_json:=<path_to_mission_geojson> \
  -p lanelet_file:=<path_to_lanelet_layer.geojson> \
  -p linestring_file:=<path_to_linestring_layer.geojson>
```

## Dependencies

- ROS 2 Humble
- [BT.CPP v3](https://github.com/BehaviorTree/BehaviorTree.CPP)
- `nav2_msgs`, `rclcpp_action`, `tf2_geometry_msgs`
- `vision_msgs` (perception message types)
- `nlohmann-json3-dev`

## Related packages

- `robotaksi_perception` — YOLO-based traffic sign and light detection
- `robotaksi_navigation` — Nav2 stack configuration
- `robotaksi_localization` — GPS/IMU-based localization

## Status

Under active development. Dynamic/static obstacle avoidance and parking-slot occupancy detection are implemented at the node level but await real sensor integration.
