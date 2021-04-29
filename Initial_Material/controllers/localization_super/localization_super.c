
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define RAD2DEG(X)      X / M_PI * 180.0
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>


#define FLOCK_SIZE	1		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step

WbNodeRef robs[1];		// Robots nodes
WbFieldRef robs_trans[1];	// Robots translation fields
WbFieldRef robs_rotation[1];	// Robots rotation fields


float loc[3];
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

		sprintf(rob,"ROBOT%d", 1);
		robs[0] = wb_supervisor_node_get_from_def(rob);
		robs_trans[0] = wb_supervisor_node_get_field(robs[0],"translation");
		robs_rotation[0] = wb_supervisor_node_get_field(robs[0],"rotation");
	
}

void controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
   fprintf(fp, "time; loc_x; loc_z_; theta\n");
  


}

void controller_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g;  %g; %g; %g\n",
            time, loc[0],loc[1], RAD2DEG(loc[2]));
  

}
}

/**
void compute_fitness_acc(float* fit) {


	*fit =abs(loc[0]-_odo_acc.x)+abs(loc[1]-_odo_acc.y);
	}
		
*/


/*
 * Main function.
 */
 	float fit_acc =0;
int main() 
{	// Buffer for sending data
	

            
	controller_init_log("pose.csv");
	reset();
	for(;;) {
          	     wb_robot_step(TIME_STEP);
		
		if (t % 10 == 0) {

			// Get data
			loc[0] = wb_supervisor_field_get_sf_vec3f(robs_trans[0])[0]; // X
			loc[1] = wb_supervisor_field_get_sf_vec3f(robs_trans[0])[2]; // Z
			loc[2] = wb_supervisor_field_get_sf_rotation(robs_rotation[0])[3]; // THETA
			
			controller_print_log(t/1000.0);				
			//compute_fitness_acc(&fit_acc);
			printf("performance metric : %g\n", fit_acc);
			
		}

		t += TIME_STEP;
	}
	return 0;
}
