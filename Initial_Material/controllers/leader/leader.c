/*****************************************************************************/
/* File:         leader.cc                                                   */
/* Version:      1.1 -> New double key recognition                           */
/* Date:         12-Oct-15                                                   */
/* Description:  Allows to remote control a robot using the arrow keys       */
/*                                                                           */
/* Author: 	 22-Oct-04 by nikolaus.correll@epfl.ch                       */
/* Last revision:12-oct-15 by Florian Maushart				     */
/*****************************************************************************/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "trajectories.h"
#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          150     // Minimum sensibility value
#define MAX_SENS          6096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)


#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))

/*Webots 2018b*/
WbDeviceTag dev_left_motor; //handler for left wheel of the robot
WbDeviceTag dev_right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance


WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[3];	// relative X, Z, Theta of all robots
float prev_relative_pos[3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[2];		// Speeds calculated with Reynold's rules
float relative_speed[2];	// Speeds calculated with Reynold's rules
int initialized;		// != 0 if initial positions have been received
float migr[2] = {0,25};	        // Migration vector
char* robot_name;
float theta_robots;

/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	wb_robot_init();
	emitter = wb_robot_get_device("emitter");
	  if (emitter==0) printf("missing emitter\n");
	//get motors
            dev_left_motor = wb_robot_get_device("left wheel motor");
            dev_right_motor = wb_robot_get_device("right wheel motor");
            wb_motor_set_position(dev_left_motor, INFINITY);
            wb_motor_set_position(dev_right_motor, INFINITY);
            wb_motor_set_velocity(dev_left_motor, 0.0);
            wb_motor_set_velocity(dev_right_motor, 0.0);
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);


	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id); // read robot id from the robot's name

  
	initialized = 0;		  // Set initialization to 0 (= not yet initialized)
  
        printf("Reset: robot %d\n",robot_id);
        

}
void send_ping(void){
    char out[2] = "3";

	wb_emitter_send(emitter,out,strlen(out)+1); 
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) { 
	float theta = my_position[2];
  
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
  
	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;
  
	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}

void compute_wheel_speeds(int *msl, int *msr) {
	// Compute wanted position from Reynold's speed and current location
	float x = speed[0]*cosf(my_position[2]) + speed[1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[0]*sinf(my_position[2]) + speed[1]*cosf(my_position[2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}


// the main function
int main(){ 
			// Store highest sensor value	// Array for the distance sensor readings
 	reset();			// Resetting the robot
	
	// Forever
	for(;;){
              

		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
		trajectory_1(dev_left_motor, dev_right_motor);
    
		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}