# collision-predictor

### Algorithm pseudo code:
```python
future_traj_vehicle = {}
while not collision(future_traj_vehicle):
  for veh in range(len(vehicles):
      store velocity information from last 'n' time steps: [v1, v2, .., vn] and the lane offset data: [d1, d2, .., dn]
      compute average velocity from these time steps: v_avg
      compute average deviation from center: d_avg
      get a route to trace from start to end: [p1, p2, ..., pn]
      convert this route into s-map along the curvature: [dist(p1,p2), dist(p1,p3), ..., dist(p1,pn)]
      convert the current cartesian point into frenet point: [(s1, d1)]
      get the future trajectory based on the constant velocity assumption: [(s1,d1), (s2,d2), ..., (sn,dn)]
      revert these future trajectory points into cartesian coordinates: [(x1,y1), (x2,y2), ...,(xn,yn)]
      future_traj_veh[veh] = [(x1,y1), (x2,y2), ...,(xn,yn)]
```

### Pending tasks:
1. Limit the number of vehicles: for computation, register only the vehicles which are in vicinity of the ego vehicle
2. Restart ego vehicle behavior: ego vehicle should start moving once the collision scenario is over
3. Region of interest: change the ROI according to the situation, w.r.t. ego vehicle or intersection
4. Vehicle that's not following the center lane
5. Better way of generating SD maps from the past information
6. Initialization of the future trajectories
7. Dictionary of vehicle data updates
8. Move along the trajectory rather than the normal kinematic behavior 
9. Make relevant gazebo environment
10. Add documentation for every files and their functions
