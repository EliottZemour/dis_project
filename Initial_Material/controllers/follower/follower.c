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

void update_leader_measurement(float new_leader_range, float new_leader_bearing) {
	leader_range = new_leader_range;
	leader_bearing = new_leader_bearing;
}

void compute_wheel_speeds(int nsl, int nsr, int *msl, int *msr) {
	// Define constants
	float Ku = 2.0;
	float Kw = 10.0;
	float Kb = 1.0;

	// Compute the range and bearing to the wanted position
	float x = leader_range * cosf(leader_bearing);
	float y = leader_range * sinf(leader_bearing);
	//float theta = leader_orientation;
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

void process_received_ping_messages(void) {
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double range;
	while (wb_receiver_get_queue_length(receiver) > 0) {
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2];
		double x = message_direction[0];

		theta =	-atan2(y,x);
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));
		
		
		// Get position update
		prev_relative_pos[0] = relative_pos[0];
		prev_relative_pos[1] = relative_pos[1];

		relative_pos[0] = range*cos(theta);  // relative x pos
		relative_pos[1] = -1.0 * range*sin(theta);   // relative y pos
	
		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

		relative_speed[0] = relative_speed[0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[0]-prev_relative_pos[0]);
		relative_speed[1] = relative_speed[1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[1]-prev_relative_pos[1]);		
		 
		wb_receiver_next_packet(receiver);
	}
}

int main(){
	int msl,msr;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	float *rbbuffer;                  // buffer for the range and bearing
	int i, initialized;
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double range;
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
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y_init = message_direction[2];
		double x_init = message_direction[0];

		double theta_init =	-atan2(y_init,x_init);
		theta_init += my_position[2]; // find the relative theta;
		double range_init = sqrt((1/message_rssi));
		relative_pos_init[0] = range_init*cos(theta_init);  // relative x pos
		relative_pos_init[1] = -1.0 * range_init*sin(theta_init);   // relative y pos
		goal_range = sqrt(relative_pos_init[0]*relative_pos_init[0] + relative_pos_init[1]*relative_pos_init[1]);
		goal_bearing = -atan2(relative_pos_init[0],relative_pos_init[1]);

		wb_receiver_next_packet(receiver);
		}
	}
	msl=0; msr=0;  

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
		process_received_ping_messages();
		new_leader_range = sqrt(relative_pos[0]*relative_pos[0] + relative_pos[1]*relative_pos[1]);
		new_leader_bearing = -atan2(relative_pos[0],relative_pos[1]);
		update_leader_measurement(new_leader_range, new_leader_bearing);

	          
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
  
