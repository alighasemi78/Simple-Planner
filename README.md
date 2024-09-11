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
In order to listen to the initial pose selected in RViz, we need a node handle to subscribe to the topic `/initialpose`. For the goal pose, we need another node handle to subscribe to the topic `/move_base_simple/goal`. For the map, we need another node handle to subscribe to the topic `/map`.

After receiving all these information, we need to start working on the path planning algorithm which in our case is A*.

The first step is to convert the grid coordinates of the initial and goal poses into their corresponding world coordinates compatible with the map data. This is done with the following formula:

```
world = (grid - map_origin) / map_resolution
```

Once we have the world coordinates for both the initial and goal poses, we can initiate the A* search algorithm by calling the `AStarSearch` function.

A `priority_queue` called `openList` is created to store nodes (possible waypoints on the map). This queue is ordered by the nodes’ `fCost` (the sum of the movement cost `gCost` and heuristic `hCost`).

Two 2D arrays, `parentX` and `parentY`, are created to keep track of each node’s parent, allowing us to reconstruct the path later.

Another 2D array, `closedList`, is used to keep track of nodes that have already been visited and processed.
	
A `Node` object is created for the start position, with its `gCost` (cost from start to current node) set to 0 and `hCost` (estimated cost to the goal using the heuristic function) calculated using the Manhattan distance. This node is then added to the openList.

The algorithm enters a loop that continues until `openList` is empty (indicating all possible paths have been evaluated) or the goal has been reached. In each iteration:
* The node with the lowest `fCost` (best candidate) is popped from the `openList`. This node becomes the current node.
* If the current node’s coordinates match the goal coordinates, it means the goal has been reached, and the path reconstruction starts.
* If not, the algorithm marks the current node as visited by setting `closedList[current.y][current.x] = true`.

For the current node, the algorithm checks its neighboring nodes (up, down, left, right, and diagonals). For each neighbor:
* It checks if the neighbor is within the map boundaries and if it’s a valid (non-obstacle) position using the `isValid` function.
* If the neighbor is valid and hasn’t been visited yet (`closedList[newY][newX]` is `false`), the algorithm calculates the neighbor’s `gCost` (distance from the start) and `hCost` (estimated distance to the goal).
* The neighbor is then added to the `openList` with its `parentX` and `parentY` set to the coordinates of the current node, so we can trace the path back later.

Once the goal is reached, the algorithm uses the `parentX` and `parentY` arrays to trace back from the goal to the start, reconstructing the path.

As the path is reconstructed, it is published using `path_pub` so it can be visualized in RViz.

If the `openList` becomes empty and no path to the goal has been found, the algorithm concludes that there is no valid path.

In the project definition, it states that the cost of being in a cell closer to obstacles should be higher than other ones. This is done using the `computeObstacleDistances` function which starts from the obstacles and in a BFS fashion, assigns the distance of each empty cell to their closest obstacle.

This is then used in the A* algorithm where instead of just adding a constant cost of 1 for each movement, we also add the inverse of the corresponding distance multiplied by a multiplier. This multiplier is there to make it more important than the constant 1 cost.
