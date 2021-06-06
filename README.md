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



## Part I: Localization Techniques for Individual Robots

The code for part I can be found in Initial_Material >  controllers > localization.
To run this part, open the world file `localization.wbt`. It should have `localization_super.c` as controller in the supervisor node and  `localization_controller.c` as controller for the epuck node. This should create a csv file for the odometry, gps and kalman results: localization.csv. The supervisor writes the metric for this part in the pose.csv file.

The Matlab file that can be found at Initial_Material > matlab > plotmain_x.m should reproduce the plots presented in the report.
Note that the `linear_algebra.c/h` files are helper files for the Kalman algorithm.

## Part II: Spatial Coordination Solutions

### Flocking

* ***Obstacle***:
  The code for the flocking in the obstacle world can be found at Initial_Material >  controllers > flocking.
  To run this part, open the world file `obstacle_test.wbt`. Then select `flocking_super.c` as controller in the supervisor node and  `flocking_controller.c` as controller for the epuck nodes. The supervisor writes the metric for this part in the flocking_metric.csv file.
* ***Crossing***:
  The code for the flocking in the crossing world can be found at Initial_Material >  controllers > crossing.
  To run this part, open the world file `crossing_test.wbt`. **XAV EXPLIQUES QUOI METTRE EN SUPER ET CONTROLLER EN FONCTION DU ROBOT ID**. Then select `flocking_super.c` as controller in the supervisor node and  `flocking_controller.c` as controller for the epuck nodes.



### Formation

* ***Obstacle***:
  The code for the formation in the obstacle world can be found at Initial_Material > controllers > leader and Initial_Material >  controllers > follower.
  To run this part, open the world file `obstacle_test.wbt`. Then select `formation_super.c` as controller in the supervisor node and  `leader.c` as controller for the epuck0. Followers that are the other epucks have the controller `follower.c`. The supervisor writes the metric for this part in the formation_metric.csv file.

## Part III: Particle Swarm Optimization (PSO)

The code for the PSO part can be found at PSO > controllers. It is divided in three files that communicate with each other.
The `pso.c` file implements the main PSO loop (initialize swarm, move particles, evaluate, find best particle). The supervisor `pso_obstacle_super.c` repositions the particles at the initial position at each iteration, then send candidate solutions to robots (Reynolds rules weights & thresholds), and calculates fitness. 
The controller `pso_obstacle_controller.c` receives the Reynold weights from the supervisor, and runs the controller with those weights.

