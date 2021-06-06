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
#define MAX_SENS          4096    // Maximum sensibility value
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

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define ABS(x) ((x>=0)?(x):-(x))

/*Webots 2018b*/
WbDeviceTag dev_left_motor; //handler for left wheel of the robot
WbDeviceTag dev_right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/


int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3}; // Braitenberg Matrix


WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag emitter;		// Handle for the emitter node


float relative_pos[3];	// relative X, Z, Theta of all robots
float prev_relative_pos[3];	// Previous relative  X, Z, Theta values
float my_position[3]= {-2.9 , 0, -1.5708};     		// X, Z, Theta of the current robot with initialization
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[2];		// Speeds calculated with Reynold's rules
float relative_speed[2];	// Speeds calculated with Reynold's rules
int initialized;		// != 0 if initial positions have been received
float migr[2] = {10,0};	        // Migration vector
char buffer[255]; // Buffer for emitter

	   

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

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);

  
	initialized = 0;		  // Set initialization to 0 (= not yet initialized)
  
        printf("Reset: robot0\n");
        

}



/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void) {
	sprintf(buffer, "%f", my_position[2]);
	wb_emitter_send(emitter,buffer,strlen(buffer)); 
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
 * Used for odometry
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

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
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
          limit(msl, MAX_SPEED);
          limit(msr, MAX_SPEED);
}


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds
	
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	
	
	int sum_sensors;	// Braitenberg parameters
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value	
 	reset();			// Resetting the robot

	msl = 0; msr = 0; 
	max_sens = 0; 			// Resetting the robot
	
	// Forever
	for(;;){
                      // Braitenberg
		int sensor_nb;
		int bmsl = 0;
		int bmsr = 0;
		sum_sensors = 0;
		for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  // read sensor values and calculate motor speeds
		  distances[sensor_nb]=wb_distance_sensor_get_value(ds[sensor_nb]);
		  /* Weighted sum of distance sensor values for Braitenburg vehicle */
		  bmsr += distances[sensor_nb] * Interconn[sensor_nb];
		  bmsl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
		}
		bmsl /= 400; bmsr /= 400;        // Normalizing speeds
              
		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
                      
		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		prev_my_position[2] = my_position[2];
		update_self_motion(msl,msr);
		
                      // Migratory urge
		speed[0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT *2.0;
		speed[1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT*2.0; //y axis of webots is inverted
    
		// Compute wheels speed relatively to migratory urge
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
    
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;

		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		

		wb_motor_set_velocity(dev_left_motor, msl_w);
		wb_motor_set_velocity(dev_right_motor, msr_w);
    
		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}
