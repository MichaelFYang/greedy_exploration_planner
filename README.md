## visibility_greedy_planner

This visibility planner is more like a local greedy seach using frointer information. It basically generate a distribution of potential informative score on discretize direction. And then, guide the robot to the direction with the maximum score.

## Dependence
Please follow the instruction in [dfs_behavior_planner](https://bitbucket.org/cmusubt/dfs_behavior_planner/src/master/)

## Launch vb_greedy_planner
roslaunch vb_greedy_planner vb_greedy_planner.launch
