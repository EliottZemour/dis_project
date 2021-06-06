# ENG-466 Distributed intelligent systems: Course Project

This file aims to give an overview of the code implemented in the framework of ENG-466 course project.

#### Team members:

* _[Nicolas Marbot](https://people.epfl.ch/nicolas.marbot)_
* _[Christopher Stocker](https://people.epfl.ch/christopher.stockersalas)_
* _[Xavier Suermondt](https://people.epfl.ch/xavier.suermondt)_
* _[Eliott Zemour](https://people.epfl.ch/eliott.zemour)_

#### Structure of the directory:

```
Initial_Material/
    ├─ controllers/
    │  ├─ localization/
    │  │  ├─ localization_controller.c
    │  │  ├─ localization_super.c
    │  │  ├─ odometry.c
    │  │  ├─ odometry.h
    │  │  ├─ kalman.c
    │  │  ├─ kalman.h
    │  │  ├─ trajectories.c
    │  │  ├─ trajectories.h
    │  │  ├─ linear_algebra.c
    │  │  ├─ linear_algebra.h
    │  ├─ flocking/
    │  │  ├─ flocking_controller.c
    │  │  ├─ flocking_super.c
    │  ├─ formation/
    │  │  ├─ formation_controller.c
    │  │  ├─ formation_super.c
    │  ├─ crossing/
    │  │  ├─ crossing_left_controller.c
    │  │  ├─ crossing_left_super.c
    │  │  ├─ crossing_right_controller.c
    │  │  ├─ crossing_right_super.c
    ├─ worlds/
    │  ├─ localization.wbt
    │  ├─ crossing.wbt
    │  ├─ obstacle.wbt
    │  ├─ crossing-test.wbt
    │  ├─ obstacle-test.wbt
    
PSO/
    ├─ controllers/
    │  ├─ pso_obstacle_super.c
    │  ├─ pso_obstacle_controller
    │  ├─ pso.c
    │  ├─ pso.h
    ├─ worlds/
    │  ├─ obstacle-test-pso.wbt
    
```

```
Initial_Material/
    ├─ controllers/
       ├─ localization/
         ├─ localization_controller.c
         ├─ localization_super.c
         ├─ odometry.c
         ├─ odometry.h
         ├─ kalman.c
         ├─ kalman.h
         ├─ trajectories.c
         ├─ trajectories.h
         ├─ linear_algebra.c
         ├─ linear_algebra.h
       ├─ flocking/
          ├─ flocking_controller.c
          ├─ flocking_super.c
       ├─ formation/
          ├─ formation_controller.c
          ├─ formation_super.c
       ├─ crossing/
          ├─ crossing_left_controller.c
          ├─ crossing_left_super.c
          ├─ crossing_right_controller.c
          ├─ crossing_right_super.c
    ├─ worlds/
       ├─ localization.wbt
       ├─ crossing.wbt
       ├─ obstacle.wbt
       ├─ crossing-test.wbt
       ├─ obstacle-test.wbt
    
PSO/
    ├─ controllers/
    │  ├─ pso_obstacle_super.c
    │  ├─ pso_obstacle_controller
    │  ├─ pso.c
    │  ├─ pso.h
    ├─ worlds/
    │  ├─ obstacle-test-pso.wbt
    
```



## Part I: Localization Techniques for Individual Robots

The code for part I can be found in Initial_Material >  controllers > localization.
To run this part, open the world file `localization.wbt`. It should have `localization_super.c ` as controller in the supervisor node and  `localization_controller.c ` as controller in the epuck node. This should create csv files for the odometry results.
The Matlab file that can be found at Initial_Material > matlab > plotmain.m should reproduce the plots presented in the report.

## Part II: Spatial Coordination Solutions

### Flocking

### Formation

## Part III: Particle Swarm Optimization



