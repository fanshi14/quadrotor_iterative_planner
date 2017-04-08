# octomap_truck
Iterative generating safe control polygon for bspline trajectory.

## Run simulator
```  
roslaunch truck_server octomap_truck_server.launch debug:=true dji:=false uav_direct_start:=false landing:=true
```

1.Make uav take off and fly to specific height.
```
rostopic pub /uav_start_flag std_msgs/Empty "{}"
```

2.Make uav start to follow planned trajectory.
```
rostopic pub /task1_arrive_gps_point std_msgs/Empty "{}"
```


3.Make uav start to land (landing args should be true)
```
rostopic pub /uav_straight_lane_landing_start_flag std_msgs/Empty "{}"
```


## State list
State: 
```
0, still; 1, taking off; 2, ready to move; 3, start to track; 4, wait to land; 5, start to land; 6, wait to force land; 7, start force land; 8, during force land; 9, finish force land.
```

State 3: start to track and land in m_uav_tracking_landing_constant_speed (~0.5m/s) to (m_uav_start_landing_height_upperbound (~5m) + m_target_height), then changes to state 4 wait to land.

When publish topic "/uav_straight_lane_landing_start_flag" or GPS point is inside trigger region (to do), uav will changes to state 5, then land in m_uav_landing_constant_speed (~1.0m/s) until landing.  
