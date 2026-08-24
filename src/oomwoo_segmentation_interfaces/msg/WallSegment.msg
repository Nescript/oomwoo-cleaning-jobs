# One detected wall segment in the map frame, derived data.
# Detected walls are algorithm output, not user constraints: they are
# reproducible from the Source Map and are never persisted as-is. Converting
# one into a VirtualWall requires explicit user confirmation.
# Endpoints are map-frame coordinates in meters (same frame as the
# OccupancyGrid in the goal).
float64 x1
float64 y1
float64 x2
float64 y2
# Wall support in [0, 1]: fraction of raster evidence along the segment,
# as computed by the provider (ROSE2 Hough/structural-raster support).
float64 support
# Dominant direction of the wall in radians, map frame, [0, pi).
float64 direction_rad
