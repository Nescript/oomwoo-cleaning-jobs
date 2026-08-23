# Boolean grid in OccupancyGrid row-major order.
# 0 excludes a cell; any non-zero value includes it.
std_msgs/Header header
nav_msgs/MapMetaData info
uint8[] data
