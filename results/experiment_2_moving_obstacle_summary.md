# Experiment 2: Dynamic Obstacle Detection - Multiple Positions

## Objective
Observe how the costmap responds when obstacles are moved to different positions in real-time.

## Method
1. Placed obstacle at Position 1
2. Observed costmap update in RViz
3. Moved obstacle to Position 2
4. Observed costmap response
5. Repeated for Positions 3-4

## Key Findings
- Static map remained unchanged across all positions
- Costmap updated in real-time as obstacle moved
- Robot maintained localization on saved map
- No permanent modifications to saved map file
- System successfully detected dynamic obstacles

## Conclusion
The SLAM Toolbox localization mode + Nav2 costmap system successfully handles dynamic obstacles without modifying the static map. This confirms the viability of using this approach for real-time navigation in environments with temporary obstacles.

## Next Experiments Needed
- Measure costmap update latency
- Test with multiple simultaneous obstacles
- Test with moving obstacles at different speeds
