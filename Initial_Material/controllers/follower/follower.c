/*****************************************************************************/
/* File:         follower.c                                                  */
/* Version:      3.0                                                         */
/* Date:         02-Nov-05 -- 06-Oct-2015                                    */
/* Description:  Formation movement with E-Pucks                             */
/*                                                                           */
/* Author: 	 22-Oct-04 by nikolaus.correll@epfl.ch                       */
/* improved by Ali Marjovi 20-Oct-2014 and 06-Oct 2015			     */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS           8

/* Formation flocking parameters */
#define D                    0.20      // Distance between robots
#define AXLE_LENGTH          0.052     // Distance between wheels of robot
#define SPEED_UNIT_RADS      0.0628    // Conversion factor between speed unit to radian per second
#define WHEEL_RADIUS         0.0205    // Wheel radius in meters
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define DELTA_T			0.064	// Timestep (seconds)
/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};	
WbDeviceTag ds[NB_SENSORS];           // Handle for the infrared distance sensors
WbDeviceTag receiver;                 // Handle for the receiver node for range and bearing information


int robot_id;                       // Unique robot ID
float relative_pos[3];	// relative X, Z, Theta of all robots
float relative_pos_init[3];	// relative X, Z, Theta of all robots
float prev_relative_pos[3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3]; 
float speed[2];		// Speeds calculated with Reynold's rules
float relative_speed[2]; 
float goal_range   = 0.0;
float goal_bearing = 0.0;
float theta;
float leader_range = 0.0;
float leader_bearing = 0.0;
float leader_orientation = 0.0;
static void reset(void) {
	wb_robot_init();

	receiver    = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver,64); 
	/*Webots 2018b*/
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	/*Webots 2018b*/

	int i;

	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);      // the device name is specified in the world file
		s[2]++;                         // increases the device number
	}
	char* robot_name; robot_name=(char*) wb_robot_get_name(); 

	sscanf(robot_name,"epuck%d",&robot_id);    // read robot id from the robot's name
	printf("Reset: robot %d\n",robot_id);
}



void update_leader_measurement(float new_leader_range, float new_leader_bearing, float new_leader_orientation) {
	leader_range = new_leader_range;
	leader_bearing = new_leader_bearing;
	leader_orientation = new_leader_orientation;
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


void compute_wheel_speeds(int nsl, int nsr, int *msl, int *msr) {
	// Define constants
	float Ku = 5;
	float Kw = 10.0;
	float Kb = 1.0;


	// Compute the range and bearing to the wanted position
	float x = leader_range * cosf(leader_bearing);
	float y = leader_range * sinf(leader_bearing);
	float theta = leader_orientation;
	x += goal_range * cosf(- M_PI + goal_bearing + theta);
	y += goal_range * sinf(- M_PI + goal_bearing + theta);
	float range = sqrtf(x*x + y*y); // This is the wanted position (range)
	float bearing = atan2(y, x);    // This is the wanted position (bearing)

	// Compute forward control (proportional to the projected forward distance to the leader
	float u = Ku * range * cosf(bearing);
	// Compute rotional control
	float w = Kw * range * sinf(bearing) + Kb * leader_orientation;
	// Of course, we can do a lot better by accounting for the speed of the leader (rather than just the position)

	// Convert to wheel speeds!
	*msl = (int)((u - AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = (int)((u + AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	

}

int main(){
	int msl,msr;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int distances[NB_SENSORS];        // array keeping the distance sensor readings                 // buffer for the range and bearing
	int i, initialized;
	const double *message_direction;
	float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	double message_rssi; // Received Signal Strength indicator
	float *rbbuffer;
	float leader_heading_init;
	reset();                          // Initialization 
	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);

	//read the initial packets
	initialized = 0;
	while(!initialized){
		/* Wait until supervisor sent range and bearing information */
		while (wb_receiver_get_queue_length(receiver) == 0) {
			wb_robot_step(64); // Executing the simulation for 64ms
		}  
		if (wb_receiver_get_queue_length(receiver) > 0) {
		initialized=1;
		rbbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(rbbuffer, "%f", &leader_heading_init);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		
		double y_init = message_direction[2];
		double x_init = message_direction[0];
		double theta_init = -atan2(y_init,x_init);
		theta_init += my_position[2];
            	if (theta_init > M_PI) theta_init -= 2.0*M_PI;
            	if (theta_init < -M_PI) theta_init += 2.0 * M_PI;
		double range_init = sqrt((1/message_rssi));
		relative_pos_init[0] = range_init*cos(theta_init);  // relative x pos
		relative_pos_init[1] = -1.0 * range_init*sin(theta_init);   // relative y pos
		
		goal_range = range_init;
		goal_bearing = -atan2(relative_pos_init[0],relative_pos_init[1]);
		
		leader_range = goal_range;
		leader_bearing = goal_bearing;

		
                      printf(" goal range = %f , goal bearing = %f %d\n", goal_range, goal_bearing, robot_id);
		wb_receiver_next_packet(receiver);
		}
	}
	msl=0; msr=0;  
            double range;
            char *inbuffer;	// Buffer for the receiver node
            float leader_heading;
	for(;;){
		int sensor_nb;
		int bmsl = 0;
		int bmsr = 0;
		for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  // read sensor values and calculate motor speeds
		  distances[sensor_nb]=wb_distance_sensor_get_value(ds[sensor_nb]);
		  /* Weighted sum of distance sensor values for Braitenburg vehicle */
		  bmsr += distances[sensor_nb] * Interconn[sensor_nb];
		  bmsl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
		}
		bmsl /= 400; bmsr /= 400;        // Normalizing speeds
		
		update_self_motion(msl,msr);
		
		
            	while (wb_receiver_get_queue_length(receiver) > 0) {
            		inbuffer = (char*) wb_receiver_get_data(receiver);
            		sscanf(inbuffer, "%f", &leader_heading);
            		message_direction = wb_receiver_get_emitter_direction(receiver);
            		message_rssi = wb_receiver_get_signal_strength(receiver);
            		double y = message_direction[2];
            		double x = message_direction[0];

            		theta =	-atan2(y,x);
            		theta = M_PI/2 - theta; // find the relative theta;
            		if (theta > M_PI) theta -= 2.0*M_PI;
            		if (theta < -M_PI) theta += 2.0 * M_PI;
            		range = sqrt((1/message_rssi));
		
            		new_leader_range = range;
            		new_leader_bearing = -theta;
            		new_leader_orientation = leader_heading - my_position[2];		
		 
            		wb_receiver_next_packet(receiver);
            	}

		
		
		update_leader_measurement(new_leader_range, new_leader_bearing, new_leader_orientation);


		compute_wheel_speeds(0, 0, &msl, &msr);

		msl += bmsl;
		msr += bmsr;

		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/

		wb_robot_step(64);               // Executing the simulation for 64ms
	}
}  
  
