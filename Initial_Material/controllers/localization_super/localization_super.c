#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

 #define FLOCK_SIZE	1		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step

typedef enum { GPS, SUPERVISED } gps_mode_types;
// WbNodeRef robs[FLOCK_SIZE];		// Robots nodes

 WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
 WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
 WbDeviceTag emitter;			// Single emitter

float loc[1][3];		// Location of everybody in the flock

// #define RULE1_THRESHOLD 0.2
// #define fit_cluster_ref 0.03
// #define fit_orient_ref 1.0


 int offset;				// Offset of robots number
// float migrx, migrz;			// Migration vector
// float orient_migr; 			// Migration orientation
int t;

/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	emitter = wb_robot_get_device("emitter");
	if (emitter==0) printf("missing emitter\n");
	
	// char rob[7] = "epuck0";
	// int i;
	// for (i=0;i<FLOCK_SIZE;i++) {
		// sprintf(rob,"epuck%d",i+offset);
		// robs[i] = wb_supervisor_node_get_from_def(rob);
		// robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		// robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	// }
}


void send_init_poses(void) {
  	char buffer[255];	// Buffer for sending data
  	
  	// Get data
            loc[1][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[1])[0]; // X
            loc[1][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[1])[2]; // Z
            loc[1][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[1])[3]; // THETA
            
            // Send it out
            sprintf(buffer,"%1d#%f#%f#%f",1+offset,loc[1][0],loc[1][1],loc[1][2]);
            wb_emitter_send(emitter,buffer,strlen(buffer));
            printf("sent cordinates\n");
            // Run one step
            wb_robot_step(TIME_STEP);

}





/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	char buffer[255];	// Buffer for sending data
	int i;			// Index
	
	printf("Test \n");


	reset();

	send_init_poses();
	
	// Compute reference fitness values
	
	// float fit_cluster;			// Performance metric for aggregation
	// float fit_orient;			// Performance metric for orientation
		
	for(;;) {
		wb_robot_step(TIME_STEP);
		
		if (t % 10 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
				// Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it                   		
				sprintf(buffer,"%1d#%f#%f#%f",i+offset,loc[i][0],loc[i][1],loc[i][2]);
				wb_emitter_send(emitter,buffer,strlen(buffer));				
			}
			//Compute and normalize fitness values
			// compute_fitness(&fit_cluster, &fit_orient);
			// fit_cluster = fit_cluster_ref/fit_cluster;
			// fit_orient = 1-fit_orient/M_PI;
			printf("time:%d \n", t);			
		}
		t += TIME_STEP;
	}
}
