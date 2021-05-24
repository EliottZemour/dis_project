/*****************************************************************************/
/* File:         flocking1_supper.c                                          */
/* Version:      2.0                                                         */
/* Date:         10-Oct-14 -- 06-Oct-2015                                    */
/* Description:  The supervisor of a flock of robots which takes care of     */
/*		 sending the positions of the robots to the mates     	     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi 				     */
/*		 initiallz developed by Nicolas Correll    		     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define ROBOTS 5
#define TIME_STEP 16	

static WbNodeRef robs[ROBOTS];
static WbFieldRef robs_translation[ROBOTS];
static WbFieldRef robs_rotation[ROBOTS];
WbDeviceTag emitter_device;

float loc[ROBOTS][3];

int t;

float global_x,global_z;
float rel_x[ROBOTS-1];
float rel_z[ROBOTS-1];

float center[2] = {0, 0};
float center_old[2] = {0, 0};

float d_max = 6.28 * 0.020 * 0.016;

/* Good relative positions for each robot */
double good_rp[ROBOTS][2] = { {0.0,0.0}, {-0.15,-0.15}, {0.0,-0.30}, {0.15,-0.15}, {0.0,-2.45} };

float dfo_metric, v_metric; // The elements to multiply for the formation metric

void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck1";
	char emitter0[8] = "emitter";
	int i;
	robs[0] = wb_supervisor_node_get_from_def(rob);
	robs_translation[0] = wb_supervisor_node_get_field(robs[0],"translation");
	robs_rotation[0] = wb_supervisor_node_get_field(robs[0],"rotation");

	rob[5]++;
	for (i=1;i<ROBOTS;i++) {
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_translation[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");    
		rob[5]++;
	}
	emitter_device = wb_robot_get_device(emitter0);
}

void compute_dist_metric() {
	dfo_metric = 0.0;
	int i;
	for (i=1; i<ROBOTS; i++) {
		dfo_metric += sqrt(pow(rel_x[i-1]-good_rp[i][0],2) + pow(rel_z[i-1]-good_rp[i][1],2));
	}
	dfo_metric = 1.0 + dfo_metric / (ROBOTS-1);
	dfo_metric = 1.0 / dfo_metric;
}

void compute_formation_center() {
    int i;
    for (i=0; i< ROBOTS; i++) {
        center[0] += loc[i][0];
        center[1] += loc[i][1];
    }
    center[0] /= ROBOTS;
    center[1] /= ROBOTS;
}

void compute_veloc_metric() {
      v_metric = sqrtf(powf(center_old[0]-center[0],2.0) + powf(center_old[1]-center[1],2.0));
      v_metric /= d_max;
}

int main(int argc, char *args[]) {

	float fit_formation;
	int i;
	int print_enabled = 0;

	if (argc > 1) {
		print_enabled = atoi(args[1]);
		printf("Print: %d\n", print_enabled);
	}

	reset();

	for(;;) { 
		wb_robot_step(TIME_STEP); /* run one step */
		
		if (t % 10 == 0) {
			center_old[0] = center[0];
			center_old[1] = center[1];
			for (i=1;i<ROBOTS;i++) {
				/* Get data */
				loc[0][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[0];
				loc[0][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[2];
				loc[0][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[0])[3];
				
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3];

				/* Find global relative coordinates */
				global_x = loc[i][0] - loc[0][0];
				global_z = loc[i][1] - loc[0][1];
				/* Calculate relative coordinates */
				rel_x[i-1] = -global_x*cos(loc[i][2]) + global_z*sin(loc[i][2]);
				rel_z[i-1] = global_x*sin(loc[i][2]) + global_z*cos(loc[i][2]);

			}
			compute_formation_center();
			compute_dist_metric();
			compute_veloc_metric();
			
			fit_formation = dfo_metric * v_metric;

			if (print_enabled)
				printf("Performance metric: %.2f\n",fit_formation);
		}
		t += TIME_STEP;	
	}
	return 0;
}
