# Experiment 1: Dynamic Obstacle Detection in Localization Mode

## Objective
Test if TurtleBot3 can detect a new obstacle without modifying the saved static map.

## Method
1. Created a SLAM map by driving robot in full circle
2. Saved the map (turtlebot_experiment_v1)
3. Switched SLAM to localization mode using saved map
4. Spawned a box obstacle in front of robot
5. Observed LIDAR and costmap behavior

## Observations
- Static map remained unchanged (white/gray pixels stayed same)
- LIDAR detected new box in real-time (red laser points)
- Robot localized correctly on saved map
- Obstacle visible in point cloud but NOT in saved map

## Conclusion
The system successfully detected dynamic obstacles without modifying the static map. This confirms that localization mode + real-time obstacle detection works as intended.

## Next Steps
- Test with multiple obstacles
- Test with moving obstacles
- Measure costmap update latency
