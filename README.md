# Simple Planner

This project is intended to use ROS in order to implement a path planning algorithm to find a suitable path between two points on a map.

## Development

To create the workspace, we first need to create a `src` folder and the calling the command `catkin_make` inside the root folder.

`map_server` was used to load the map. In this approach, we just need the image itself and a `.yaml` file describing some metadata about the map. The original image had some problems in the walls of the map (some places were not connected). To solve this issue, the transparent part of the map was replaced with a black background to ensure the restriction of the path inside the walls. This black background was created by using the following command from the `imagemagick` library:

```
convert diag_map.png -background black -alpha remove -alpha off diag_map_clean.png
```
