#include "pso.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>




#define FLOCK_SIZE				5 		// Number of robots in flock
#define TIME_STEP				64		// [ms] Length of time step

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float loc[FLOCK_SIZE][3];
float loc_old[FLOCK_SIZE][3];

#define RULE1_THRESHOLD 		0.2
#define fit_cluster_ref 		0.03
#define fit_orient_ref 			1.0
#define MAXSPEED                0.1287
#define SIZE_MAX_CLUSTER        FLOCK_SIZE+1
#define NB_MAX_CLUSTER          FLOCK_SIZE
#define END_CLUSTER_IDX         SIZE_MAX_CLUSTER
#define DH                      0.01
#define H_MAX                   5.00
#define N_PAIRS FLOCK_SIZE*(FLOCK_SIZE-1)/2
#define MIN(x, y) (((x) < (y)) ? (x) : (y))


float pair[N_PAIRS][2];
float center[2] = {0, 0};
float center_old[2] = {0, 0};
float d_max = 6.28 * 0.020 * 0.016;

float o_metric, dfl_metric, v_metric, fit_cluster;
//weights for the performance calculation
#define W_O    0.5   //orientation
#define W_C    0.0   //cohesion (not used)
#define W_V    1.0   //velocity
#define W_S    5.0   //entropy

int clusters[NB_MAX_CLUSTER][SIZE_MAX_CLUSTER]; //+1 because we want to add END_CLUSTER_IDX
float dist_robot[FLOCK_SIZE][FLOCK_SIZE];

int offset;					// Offset of robots number
float orient_migr = 0.0; 			// Migration orientation
int t;

/* PSO definitions */
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 1.2  
// 0.6 of Inertia !                     // Maximum velocity particle can attain
#define MININIT 0.0                   // Lower bound on initialization value
#define MAXINIT 1.0                    // Upper bound on initialization value
#define ITS 10                          // Number of iterations to run
#define MAX_ROB FLOCK_SIZE
#define ROBOTS FLOCK_SIZE

#define DATASIZE 7       // Number of elements in particle
#define SWARMSIZE 5                    // Number of particles in swarm

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 180                     // Number of fitness steps to run during evolution

#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8

WbDeviceTag emitter[MAX_ROB];
WbDeviceTag rec[MAX_ROB];
const double *loc_[MAX_ROB];
const double *rot[MAX_ROB];
double new_loc[MAX_ROB][3];
double new_rot[MAX_ROB][4];
float loc[FLOCK_SIZE][3];

void calc_fitness(double[DATASIZE],double*,int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);
void compute_pair_matrix();
/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck0";
    char em[] = "emitter0";
    char receive[] = "receiver0";

	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
        sprintf(rob,"epuck%d",i+offset);
        robs[i] = wb_supervisor_node_get_from_def(rob);
        robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
        loc_[i] = wb_supervisor_field_get_sf_vec3f(robs_trans[i]);
        new_loc[i][0] = loc_[i][0]; 
        new_loc[i][1] = loc_[i][1]; 
        new_loc[i][2] = loc_[i][2];
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
		loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
        rot[i] = wb_supervisor_field_get_sf_rotation(robs_rotation[i]);
        new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
        emitter[i] = wb_robot_get_device(em);
        if (emitter[i]==0) 
            printf("missing emitter %d\n",i);
        rec[i] = wb_robot_get_device(receive);
        em[7]++;
        receive[8]++;
    }
}

/*
 * Compute performance metric.
 */
 
int main(int argc, char *args[]) {
    double *weights;                         // Evolved result
    double buffer[255];
    int i,j,k;

    wb_robot_init();
    printf("Particle Swarm Optimization Super Controller\n");
    reset();
    wb_robot_step(256);

    double fit, endfit, w[DATASIZE], f;
    double bestfit, bestw[DATASIZE];

    compute_pair_matrix();

    for(i = 0; i < FLOCK_SIZE; ++i)
    	wb_receiver_enable(rec[i],32);

    // Evolve controllers
    endfit = 0.0;
    bestfit = 0.0;



    for (j=0;j<10;j++) {
    
        /* Get result of evolution */
        weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS);
    
        /* Calculate performance */
        fit = 0.0;
        for (k=0;k<DATASIZE;k++)
            w[k] = weights[k];
        
        calc_fitness(w,&f,FIT_ITS,MAX_ROB);
        fit = f;

        /* Check for new best fitness */
        if (fit > bestfit) {
            bestfit = fit;
        for (i = 0; i < DATASIZE; i++)
            bestw[i] = weights[i];
        }
        
        //printf("Performance: %.3f\n",fit);
        endfit += fit/10;

	    /* Send best controller to robots */
	    for (i=0;i<DATASIZE;i++) {
	        buffer[i] = bestw[i];
	    }
    }
    //printf("Average performance: %.3f\n",endfit);

    return 0;
}


// Randomly position specified robot
void random_pos(int rob_id) {
	wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id],"translation"), new_loc[rob_id]);
	wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id],"rotation"), new_rot[rob_id]);
}

// Distribute fitness functions among robots
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
    float n = (float) FLOCK_SIZE;
    for (i=0; i< FLOCK_SIZE; i++) {
        center[0] += loc[i][0];
        center[1] += loc[i][1];
    }
    center[0] /= n;
    center[1] /= n;
}

void compute_orient_metric() {
      o_metric = 0.0;
      int i,j,k;
      float Hdiff;
      float n = (float) N_PAIRS;
      for (k=0; k<N_PAIRS; k++) {
          i = pair[k][0];
          j = pair[k][1];
          Hdiff = ( loc[i][2] -  loc[j][2] );
          Hdiff = Hdiff > M_PI ? 2*M_PI-Hdiff : Hdiff; 
          Hdiff = Hdiff < -M_PI ? 2*M_PI+Hdiff : Hdiff; 
          //printf("Hdiff/pi  = %f\n", fabsf(Hdiff)/M_PI);
          o_metric += fabsf(Hdiff)/M_PI;
      }
      //printf("npairs = %f\n", n);
      //printf("o_metric = %f\n", o_metric);
      //printf("sum/npairs = %f\n", (o_metric/(float) N_PAIRS));
      o_metric = 1 - o_metric/n;  
}

void compute_dist_metric() {
      dfl_metric = 0.0;
      int i,j,k;
      float denominator = 1.0; 
      float n = (float) N_PAIRS;
          
      for (i=0; i<FLOCK_SIZE; i++) {
          denominator += (sqrtf(powf(loc[i][0]-center[0],2.0) + powf(loc[i][1]-center[1],2.0)))/FLOCK_SIZE;
      }
      
      float delta_x;
      
      for (k=0; k<N_PAIRS; k++) {
          i = pair[k][0];
          j = pair[k][1];
          delta_x = sqrtf(powf(loc[i][0]-loc[j][0],2.0) + powf(loc[i][1]-loc[j][1],2.0));
          dfl_metric += MIN(delta_x/0.14, 1/powf(1.14+delta_x,2.0))/n;
      }  
      //printf("denominator = %f\n", denominator);
      dfl_metric /= denominator;
}

void compute_veloc_metric() {
      v_metric = (sqrtf(powf(center_old[0]-center[0],2.0) + powf(center_old[1]-center[1],2.0)));
      v_metric /= d_max;
}

void compute_fitness() {
            int i;
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
	// printf("O: %f \t dfl: %f \t v:%f \n",o_metric,dfl_metric,v_metric);
	fit_cluster =o_metric * dfl_metric * v_metric;
}

// Distribute fitness functions among robots
void calc_fitness(double weights[DATASIZE], double* fit, int its, int numRobs) {
    double buffer[255];
    int i;
    double perf = 0;
    int nbMeasure = 0;
    double* rbuffer;
    double local_perf = 0;

    /* Send data to robots */
    for(i = 0; i < FLOCK_SIZE; ++i)
	   	random_pos(i);

	/* Send best controller to robots */
	for (i=0;i<DATASIZE;i++)
	    buffer[i] = weights[i];

    for (i=0;i<ROBOTS;i++) {
        wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
    }

	  /* Wait for response */
		while (wb_receiver_get_queue_length(rec[0]) == 0) {
			compute_fitness();
			if(fit_cluster > 0) {
				perf += fit_cluster;
				++nbMeasure;
				//printf("%f\n", local_perf);
			  }
			wb_robot_step(64);
		}
		/* Get fitness values */
		for (i=0;i<FLOCK_SIZE;i++) {
			rbuffer = (double *)wb_receiver_get_data(rec[i]);
			wb_receiver_next_packet(rec[i]);
		}
    printf(" fit cluster %f \n",fit_cluster);
    *fit = perf / nbMeasure;

}

    // Evolution fitness function
void fitness(double weights[], double* fit, int neighbors[SWARMSIZE][SWARMSIZE]) {
    
    calc_fitness(weights,fit,FIT_ITS,ROBOTS);
    #if NEIGHBORHOOD == RAND_NB
        nRandom(neighbors,2*NB);
    #endif
    #if NEIGHBORHOOD == NCLOSE_NB
        nClosest(neighbors,2*NB);
    #endif
    #if NEIGHBORHOOD == FIXEDRAD_NB
        fixedRadius(neighbors,RADIUS);
    #endif
}

/* Get distance between robots */
double robdist(int i, int j) {
    return sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][2]-loc[j][2],2));
}

/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {
    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

    /* Clear old neighbors */
    for (j = 0; j < ROBOTS; j++)
        neighbors[i][j] = 0;

    /* Set new neighbors randomly */
    for (j = 0; j < numNB; j++)
        neighbors[i][(int)(SWARMSIZE*rnd())] = 1;
    }
}

/* Choose the n closest robots */
void nClosest(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

    int r[numNB];
    int tempRob;
    double dist[numNB];
    double tempDist;
    int i,j,k;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Clear neighbors */
        for (j = 0; j < numNB; j++)
            dist[j] = 100000000;

        /* Find closest robots */
        for (j = 0; j < ROBOTS; j++) {

            /* Don't use self */
            if (i == j)
                continue;

            /* Check if smaller distance */
            if (dist[numNB-1] > robdist(i,j)) {
                dist[numNB-1] = robdist(i,j);
                r[numNB-1] = j;

                /* Move new distance to proper place */
                for (k = numNB-1; k > 0 && dist[k-1] > dist[k]; k--) {
                    tempDist = dist[k];
                    dist[k] = dist[k-1];
                    dist[k-1] = tempDist;
                    tempRob = r[k];
                    r[k] = r[k-1];
                    r[k-1] = tempRob;
                }
            }
        }

        /* Update neighbor table */
        for (j = 0; j < ROBOTS; j++)
            neighbors[i][j] = 0;
        for (j = 0; j < numNB; j++)
            neighbors[i][r[j]] = 1;
    }

}

/* Choose all robots within some range */
void fixedRadius(int neighbors[SWARMSIZE][SWARMSIZE], double radius) {
    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {
        /* Find robots within range */
        for (j = 0; j < ROBOTS; j++) {
            if (i == j) 
                continue;
            if (robdist(i,j) < radius) 
                neighbors[i][j] = 1;
            else 
                neighbors[i][j] = 0;
        }
    }
}

void step_rob() {
    wb_robot_step(64);
} 
