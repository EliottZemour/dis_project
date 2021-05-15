
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	5		// Number of robots in flock
#define TIME_STEP	16		// [ms] Length of time step

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	           // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
//WbDeviceTag receiver;			// Single emitter

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define RULE1_THRESHOLD 0.40
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0

#define N_PAIRS FLOCK_SIZE*(FLOCK_SIZE-1)/2

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

float pair[N_PAIRS][2];
float center[2] = {0, 0};
float center_old[2] = {0, 0};
float d_max = 6.28 * 0.020 * 0.016;

int offset;				// Offset of robots number
float migrx, migrz;			// Migration vector
float orient_migr; 			// Migration orientation
int t;

float o_metric, dfl_metric, v_metric; // The elements to multiply for the perf. metric

/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	//receiver = wb_robot_get_device("receiver");
	
	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}



void compute_pair_matrix() {
    int i,j,k;
    k = 0;
    for (i=0; i<FLOCK_SIZE-1; i++) {
        for (j=i+1; j<FLOCK_SIZE; j++) {
            pair[k][0] = i;
            pair[k][1] = j;
            k += 1;
        }
    }
}

void compute_flock_center() {
    int i;
    for (i=0; i< FLOCK_SIZE; i++) {
        center[0] += loc[i][0];
        center[1] += loc[i][1];
    }
    center[0] /= FLOCK_SIZE;
    center[1] /= FLOCK_SIZE;
}

void compute_orient_metric() {
      o_metric = 0.0;
      int i,j,k;
      float Hdiff;
      for (k=0; k<N_PAIRS; k++) {
          i = pair[k][0];
          j = pair[k][1];
          Hdiff = ( loc[i][2] -  loc[j][2] );
          Hdiff = Hdiff > M_PI ? 2*M_PI-Hdiff : Hdiff; 
          o_metric -= fabsf(Hdiff)/M_PI;
      }
      o_metric = 1 - o_metric/N_PAIRS;
}

void compute_dist_metric() {
      dfl_metric = 0.0;
      int i,j,k;
      float denominator = 1.0; 
          
      for (i=0; i<FLOCK_SIZE; i++) {
          denominator += (sqrtf(powf(loc[i][0]-center[0],2.0) + powf(loc[i][1]-center[1],2.0)))/FLOCK_SIZE;
      }
      
      float delta_x;
      
      for (k=0; k<N_PAIRS; k++) {
          i = pair[k][0];
          j = pair[k][1];
          delta_x = sqrtf(powf(loc[i][0]-loc[j][0],2.0) + powf(loc[i][1]-loc[j][1],2.0));
          dfl_metric += MIN(delta_x/0.3, 1/powf(1.3+delta_x,2.0))/N_PAIRS;
      }  
      dfl_metric /= denominator;
}


void compute_veloc_metric() {
      v_metric = (sqrtf(powf(center_old[0]-center[0],2.0) + powf(center_old[1]-center[1],2.0)));
      v_metric /= d_max;
}




/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;	// Index
           //printf("just got here");
	offset = 0.0;
	migrx = 0;
	migrz = -10;
	//migration goal point comes from the controller arguments. It is defined in the world-file, under "controllerArgs" of the supervisor.
	printf("Migratory instinct : (%f, %f)\n", migrx, migrz);

	
	orient_migr = -atan2f(migrx,migrz);
	if (orient_migr<0) {
		orient_migr+=2*M_PI; // Keep value within 0, 2pi
	}
	
	compute_pair_matrix();
	reset();
          
	
	// Compute reference fitness values
	
	float fit_cluster;			// Performance metric for flocking
		
	for(;;) {
		wb_robot_step(TIME_STEP);
		if (t % 10 == 0) {
			center_old[0] = center[0];
			center_old[1] = center[1];
			for (i=0;i<FLOCK_SIZE;i++) {
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
			
			}
			compute_flock_center();
			compute_orient_metric();
			compute_dist_metric();
			compute_veloc_metric();
			
			fit_cluster = o_metric * dfl_metric * v_metric;
			printf("time:%d, Topology Performance: %f\n", t, fit_cluster);			
		}
		t += TIME_STEP;
	}
}