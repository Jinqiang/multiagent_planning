## Instructions (ACL)
Forked and modified from https://github.com/carlosluisg/multiagent_planning
Main changes are the creation of functions to avoid repeating code between different files. 

Tested in MATLAB2018a. 

Instructions (in MATLAB)
```
cd ./dmpc/matlab
Run the file dmpc_soft_bound2.m (results will appear in result_dmpc.txt, that file should exist beforehand)
```

```
cd ./dec-iSCP
Run the file dec-iSCP.m (results will appear in result_deciSCP.txt, that file should exist beforehand)
```

Below is the original Readme.md

# multiagent_planning
This github repo features the implementation of 3 optimization techniques for multiagent trajectory generation.

## cup_SCP
A centralized approach based on sequential convex programming. 

Paper: [Generation of collision-free trajectories for a quadrocopter fleet: A sequential convex programming approach](https://flyingmachinearena.org/wp-content/publications/2012/AugugliaroIROS2012.pdf)

## dec_iSCP
A decoupled approach based on incremental sequential convex programming.

Paper: [Decoupled Multiagent Path Planning via Incremental Sequential Convex Programming](http://markjcutler.com/papers/Chen15_ICRA.pdf)

## dmpc
A distributed approach based on distributed model predictive control.

Paper: [Trajectory Generation for Multiagent Point-To-Point Transitions via Distributed Model Predictive Control](https://arxiv.org/abs/1809.04230)


