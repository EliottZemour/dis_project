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

/* Added supervisor for the metric */
#include <webots/supervisor.h>
/* =============================== */


#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  16	  // [ms] Length of time step

#define AXLE_LENGTH 		0.057	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS	0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.020	// Wheel radius (meters)
#define DELTA_T		0.016	// Timestep (seconds)

#define MARGINAL_THRESHOLD  0.6       // Robots only flock with the neighbors which are closer than this threshold
#define RULE1_THRESHOLD     0.40      // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        0.99      // Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD     0.01      // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.15/10) // Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT        (0.05/10) // Weight of consistency rule. default 1.0/10
#define MIGRATION_WEIGHT    (0.03/10) // Wheight of attraction towards the common goal. default 0.01/10
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
float migr[2] = {-10,0};	        // Migration vector
char* robot_name;


WbNodeRef robs[FLOCK_SIZE];	    // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];    // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields

float loc[FLOCK_SIZE][3];	    // Location of everybody in the flock (supervisor info)

#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0


int offset;				// Offset of robots number
float migrx, migrz;			// Migration vector
float orient_migr; 			// Migration orientation

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
	
	char rob[7] = "epuck0";
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		//robs[i] = wb_supervisor_node_get_from_def(rob);
		//robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		//robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
  
	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}
  
        printf("Reset: robot %d\n",robot_id_u);
        
        migr[0] = -10;
        migr[1] = 0;
}


/*
 * Compute performance metric.
 */
void compute_fitness(float* fit_c, float* fit_o) {
	*fit_c = 0; *fit_o = 0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float angle_diff;
	int i; int j;
	for (i=0;i<FLOCK_SIZE;i++) {
		for (j=i+1;j<FLOCK_SIZE;j++) {	
			// Distance measure for each pair ob robots
			*fit_c += fabs(sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][1],2))-RULE1_THRESHOLD*2);
		}

		// Angle measure for each robot
		angle_diff = fabsf(loc[i][2]-orient_migr);
		*fit_o += angle_diff > M_PI ? 2*M_PI-angle_diff : angle_diff;
	}
	*fit_c /= FLOCK_SIZE*(FLOCK_SIZE+1)/2;
	*fit_o /= FLOCK_SIZE;
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
		speed[robot_id][0] += (migr[1]-my_position[0]) * MIGRATION_WEIGHT;
		speed[robot_id][1] -= (migr[0]-my_position[1]) * MIGRATION_WEIGHT; // y axis of webots is inverted
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


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
	
 	reset();			// Resetting the robot

	msl = 0; msr = 0; 
	max_sens = 0; 
	
	//trajectory_1(dev_left_motor, dev_right_motor);
	
	// Forever
	for(;;){

		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
                
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) {
			distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=150; bmsr+=150;
              
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
                  
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(dev_left_motor, msl_w);
		wb_motor_set_velocity(dev_right_motor, msr_w);
		
		
                  	float fit_cluster; // Performance metric for aggregation
            	float fit_orient;  // Performance metric for orientation
            	
            	for (i=0;i<FLOCK_SIZE;i++) {
			// Get data
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
		}
		
		//Compute and normalize fitness values
		compute_fitness(&fit_cluster, &fit_orient);
		fit_cluster = fit_cluster_ref/fit_cluster;
		fit_orient = 1-fit_orient/M_PI;
		printf("Flocking Performance: %f\n", fit_cluster);	
    
		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  
  
