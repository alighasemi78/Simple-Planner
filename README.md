# Simple Planner

This project is intended to use ROS in order to implement a path planning algorithm to find a suitable path between two points on a map.

## Development

To create the workspace, we first need to create a `src` folder and the calling the command `catkin_make` inside the root folder.

`map_server` was used to load the map. In this approach, we just need the image itself and a `.yaml` file describing some metadata about the map. The original image had some problems in the walls of the map (some places were not connected). To solve this issue, the transparent part of the map was replaced with a black background to ensure the restriction of the path inside the walls. This black background was created by using the following command from the `imagemagick` library:

```
convert diag_map.png -background black -alpha remove -alpha off diag_map_clean.png
```

In order to create a node that handles the path planning algorithm, we need to create a package by calling this command inside the `src` folder:

```
catkin_create_pkg path_planner roscpp nav_msgs geometry_msgs
```

This creates a package named `path_planner` which depends on `roscpp`, `nav_msgs`, and `geometry_msgs`.

Now we can make the node `path_planner_node.cpp` inside the `src` folder of the created package.

We need to connect the node to its package's `CMakeLists.txt` file by uncommenting the following lines:

```
add_executable(${PROJECT_NAME}_node src/path_planner_node.cpp)
```
```
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
```
In order to listen to the initial pose selected in RViz, we need a node handle to subscribe to the topic `/initialpose`. For the goal pose, we need another node handle to subscribe to the topic `/move_base_simple/goal`.
