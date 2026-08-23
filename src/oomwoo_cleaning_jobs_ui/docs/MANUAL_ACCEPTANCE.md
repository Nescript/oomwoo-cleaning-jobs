# Phase 1 GUI manual acceptance

Prerequisite: build all segmentation packages and run `ros2 launch oomwoo_rose2 rose2.launch.py`.

1. Open a known nav2 `map.yaml`; confirm occupied/unknown cells are not paintable and the map's vertical orientation is correct.
2. Generate candidates; confirm the GUI stays responsive while ROSE2 runs and reports an explicit error if the action server is unavailable. With a Region selected, paint, erase, create, rename, delete, merge, and split; confirm the preemption prompt and the yellow unassigned overlay.
3. Save a draft, restart the application, and reopen the same map; confirm the draft is restored.
4. Validate and publish; confirm errors block publishing, warnings are shown but allow publishing, and the version increments.
5. Click "Start /map", with a transient-local publisher publishing the map before the application starts; confirm the retained map is received. Change one cell and confirm the prompt to replace, without migrating/reusing the old region set. Also replace the map while segmentation is running and confirm the stale result is rejected rather than attached to the new map. Close the window and confirm the subscription and ROS executor stop.
6. Enter at least three map-frame `x,y` points via "Add Keepout"; enter two points and a width via "Add Virtual Wall". Confirm the magenta overlay is visible and Regions are immediately clipped; after deleting a constraint, confirm cleanable space is restored but clipped Region cells are not revived.
