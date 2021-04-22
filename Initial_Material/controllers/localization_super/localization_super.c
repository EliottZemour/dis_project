
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

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields


float loc[FLOCK_SIZE][3];
#define RULE1_THRESHOLD 0.2		// Location of everybody in the flock
static FILE *fp;

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

	char rob[7] = "ROBOT1";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"ROBOT%d", 1);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


// void send_init_poses(void) {
  	// char buffer[255];	// Buffer for sending data
	// int i;
         
	// for (i=0;i<FLOCK_SIZE;i++) {
		//Get data
		// loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
		// loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
		// loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

		//Send it out
		// sprintf(buffer,"%1d#%f#%f#%f",i+offset,loc[i][0],loc[i][1],loc[i][2]);

		//Run one step
		// wb_robot_step(TIME_STEP);
	// }
// }

void controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
   //fprintf(fp, "time; loc_x; loc_z_; theta\n");
  


}

void controller_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g;  %g; %g; %g\n",
            time, loc[0][0],loc[0][1], loc[0][2]);
  

}
}



/*
 * Main function.
 */
 
int main() 
{	// Buffer for sending data
	int i;			// Index
	

            
			controller_init_log("pose.csv");
	reset();
	for(;;) {
		wb_robot_step(TIME_STEP);
		
		if (t % 10 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {

				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
				// Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it                   		
				//sprintf(buffer,"%1d#%f#%f#%f##%f#%f",i+offset,loc[i][0],loc[i][1],loc[i][2], migrx, migrz);
				//wb_emitter_send(emitter,buffer,strlen(buffer));
				controller_print_log(t);				
			}
			
		}

		t += TIME_STEP;
	}
	return 0;
}
