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

#define NB_SENSORS	  		8	  // Number of distance sensors
#define NB_SENSOR 			8
#define DATASIZE 2*NB_SENSORS + 2    // Number of elements in particle [PSO]
#define MIN_SENS          150     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
#define MAX_ACC 800
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  16	  // [ms] Length of time step


#define AXLE_LENGTH 		0.057	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS	0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.020	// Wheel radius (meters)
#define DELTA_T		0.016	// Timestep (seconds)

#define MAX_DIFF 6.28 * 0.020 * 0.016

#define MARGINAL_THRESHOLD  0.6       // Robots only flock with the neighbors which are closer than this threshold
#define RULE1_THRESHOLD     0.3      // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        0.99      // Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD     0.05      // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.15/10) // Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT        (0.15/10) // Weight of consistency rule. default 1.0/10
#define MIGRATION_WEIGHT    (0.05/10) // Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATORY_URGE 1              // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))

/*Webots 2018b*/
WbDeviceTag dev_left_motor; //handler for left wheel of the robot
WbDeviceTag dev_right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance


WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {10,0};	        // Migration vector
char* robot_name;

float theta_robots[FLOCK_SIZE];


/*
 * Declaration of functions before the main
 */
 
void limit(int *number, int limit);
void update_self_motion(int msl, int msr);
void compute_wheel_speeds(int *msl, int *msr);
void reynolds_rules();
void send_ping(void);
void process_received_ping_messages(void);
float s(float v);
double fitfunc(double weights[DATASIZE],int its);


/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	wb_robot_init();
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	
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
	int time_step = wb_robot_get_basic_time_step();
	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],time_step);

	wb_receiver_enable(receiver,time_step);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
  
	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}
  
        printf("Reset: robot %d\n",robot_id_u);
        
        migr[0] = -5;//-10;
        migr[1] = -10;
}


int main() {
    double buffer[255];
    double *rbuffer;
    double fit;
    int i;
    
    wb_robot_init(); 
    reset();

    for(i=0;i<NB_SENSOR;i++) {
        wb_distance_sensor_enable(ds[i],64);
    }
    wb_receiver_enable(receiver,32);

    wb_robot_step(TIME_STEP);
	
	while (1) {
        // Wait for data
        while (wb_receiver_get_queue_length(receiver) == 0) {
            wb_robot_step(TIME_STEP);
        }
        printf("out of while\n");
        rbuffer = (double *)wb_receiver_get_data(receiver);
        
		fit = fitfunc(rbuffer,rbuffer[DATASIZE]);
		buffer[0] = fit;
		wb_emitter_send(emitter,(void *)buffer,sizeof(double));


        wb_receiver_next_packet(receiver);
    }
	
	return 0;	
}

/* 
 * Find the fitness for obstacle avoidance of the passed controller */
double fitfunc(double weights[DATASIZE],int its) {

    int bmsl, bmsr, sum_sensors;
    int msl, msr;
    int max_sens;
    double ds_value[NB_SENSOR];
    int i,j,k;

    // Fitness variables
    double fit_speed;           // Speed aspect of fitness
    double fit_diff;            // Speed difference between wheels aspect of fitness
    double fit_sens;            // Proximity sensing aspect of fitness
    double sens_val[NB_SENSOR]; // Average values for each proximity sensor
    double fitness;             // Fitness of controller

    // Initially no fitness measurements
    fit_speed = 0.0;
    fit_diff = 0.0;
    for (i=0;i<NB_SENSOR;i++) {
        sens_val[i] = 0.0;
    }
    fit_sens = 0.0;

    
	msl = 0; msr = 0;
	max_sens = 0; 

    // Evaluate fitness repeatedly
    for (j=0;j<its;j++) {
		
		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0; 
		
		for (k=0; k<NB_SENSORS; ++k) {
			ds_value[k] = wb_distance_sensor_get_value(ds[k]);
			
			sum_sensors += ds_value[i]; // Add up sensor values
			max_sens = max_sens>ds_value[i]?max_sens:ds_value[i]; // Check if new highest sensor value
			
			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += weights[i] * ds_value[i];
			bmsl += weights[i+NB_SENSORS] * ds_value[i];
		}
		
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=weights[2*NB_SENSORS]; bmsr+=weights[2*NB_SENSORS+1];
		
		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		update_self_motion(msl,msr);
		
		process_received_ping_messages();

		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);
    
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
    
		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);
		
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
		
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
		
        float msl_w = msl*MAX_SPEED_WEB/1000;
        float msr_w = msr*MAX_SPEED_WEB/1000; 
        wb_motor_set_velocity(dev_left_motor, (int)msl_w);
        wb_motor_set_velocity(dev_left_motor, (int)msr_w);
        wb_robot_step(128); // run one step

        // Get current fitness value

        // Average speed
        fit_speed += (fabs(msl) + fabs(msr))/(2.0*MAX_SPEED);
        // Difference in speed
        fit_diff += fabs(msl - msr)/MAX_DIFF;
        // Sensor values
        for (i=0;i<NB_SENSOR;i++) {
            sens_val[i] += ds_value[i]/MAX_SENS;
        }
    }

    // Find most active sensor
    for (i=0;i<NB_SENSOR;i++) {
        if (sens_val[i] > fit_sens) fit_sens = sens_val[i];
    }
    // Average values over all steps
    fit_speed /= its;
    fit_diff /= its;
    fit_sens /= its;

    // Better fitness should be higher
    fitness = fit_speed*(1.0 - sqrt(fit_diff))*(1.0 - fit_sens);

    return fitness;
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

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) {
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates

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

void reynolds_rules() {
  int i, j, k;                         // Loop counters
  float rel_avg_loc[2] = {0, 0};    // Flock average positions
  float rel_avg_speed[2] = {0, 0};  // Flock average speeds
  float cohesion[2] = {0, 0};  
  float dispersion[2] = {0, 0};
  float consistency[2] = {0, 0};
  float dist = 0.0;
  int n_robots = 0;            // Number of robots in range

  /* Compute averages over the whole flock */
  for (i = 0; i < FLOCK_SIZE; i++) {
    if (i != robot_id) {
      dist = sqrt(pow(relative_pos[i][0],2.0)+pow(relative_pos[i][1],2.0));

      if (dist < MARGINAL_THRESHOLD) {
        for (j = 0; j < 2; j++) {
          rel_avg_speed[j] += speed[i][j];
          rel_avg_loc[j] += relative_pos[i][j];
        }
        n_robots = n_robots + 1;
      }
    }
  }


  for (j = 0; j < 2; j++) {
    if (n_robots > 1) {
      rel_avg_loc[j] /= (n_robots - 1);
      rel_avg_speed[j] /= (n_robots - 1);
    }
  }

  /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
  for (j = 0; j < 2; j++) {
    if (fabs(rel_avg_loc[j]) > RULE1_THRESHOLD) {
      cohesion[j] = rel_avg_loc[j];
    }
  }

  /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
  for (k = 0; k < FLOCK_SIZE; k++) {
    if (k != robot_id) {
      if (sqrt(pow(relative_pos[k][0],2)+pow(relative_pos[k][1],2)) < RULE2_THRESHOLD) {
        for (j = 0; j < 2; j++) {
            dispersion[j] -= relative_pos[k][j];
        }
      }
    }
  }

  /* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
  for (j = 0; j < 2; j++) {
    consistency[j] = rel_avg_speed[j];
  }

  // aggregation of all behaviors with relative influence determined by weights
  for (j = 0; j < 2; j++) {
    speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
    speed[robot_id][j] += dispersion[j] * RULE2_WEIGHT;
    speed[robot_id][j] += consistency[j] * RULE3_WEIGHT;
  }
  speed[robot_id][1] *= -1;  // y axis of webots is inverted
  
  if(MIGRATORY_URGE == 0){
	  speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
	  speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
	} else {
	//printf("my pos: %g %g\n", my_position[0], my_position[1]);
		speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
		speed[robot_id][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT; // y axis of webots is inverted
	}
}




/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter,out,strlen(out)+1); 
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void) {
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2];
		double x = message_direction[1];

		theta =	-atan2(y,x);
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));
		

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		
		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos
          	if (other_robot_id==0) {
		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
                  }
		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);		
		 
		wb_receiver_next_packet(receiver);
	}
}


// S-function to transform v variable to [0,1]
float s(float v) {
  if (v > 5)
    return 1.0;
  else if (v < -5)
    return 0.0;
  else
    return 1.0/(1.0 + exp(-1*v));
    }
