
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define RAD2DEG(X)      X / M_PI * 180.0
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>


#define FLOCK_SIZE	1		// Number of robots in flock

#define TIME_STEP	16		// [ms] Length of time step

WbNodeRef robs[1];		// Robots nodes
WbFieldRef robs_trans[1];	// Robots translation fields
WbFieldRef robs_rotation[1];	// Robots rotation fields
WbDeviceTag receiver;             // Supervisor receiving devices (Communication from epucks)

float loc[3];
static FILE *fp;
int t;


/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();
            receiver = wb_robot_get_device("receiver");
            wb_receiver_enable(receiver,16); // enables the receiver with a sampling period of 64ms
	char rob[7] = "ROBOT1";

	
	/**
	Get information from node fields for true positions
	*/

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


/** Function to be defined to calculate the performance metric
void compute_fitness_acc(float* fit) {


	*fit =abs(loc[0]-_odo_acc.x)+abs(loc[1]-_odo_acc.y);
	}
		
*/


/*
 * Main function.
 */
 	float fit_acc =0;
int main() 

{	
	char *inbuffer;
           float rob_time, rob_gpsx, rob_gpsy, rob_gpsz, rob_odo_accx,rob_odo_accy, rob_odo_acch, rob_odo_encx, rob_odo_ency, rob_odo_ench;

            
	controller_init_log("pose.csv");
	reset();
	for(;;) {
          	     wb_robot_step(TIME_STEP);
		
		if (t % 10 == 0) {

			// Get data
			loc[0] = wb_supervisor_field_get_sf_vec3f(robs_trans[0])[0]; // X
			loc[1] = wb_supervisor_field_get_sf_vec3f(robs_trans[0])[2]; // Y
			loc[2] = wb_supervisor_field_get_sf_rotation(robs_rotation[0])[3]; // THETA
			
			controller_print_log(t/1000.0);				
			
	
			
		}
		inbuffer = (char*) wb_receiver_get_data(receiver);
			sscanf(inbuffer,"%f %f %f %f %f %f %f %f %f %f",&rob_time, &rob_gpsx, &rob_gpsy, &rob_gpsz, &rob_odo_accx, &rob_odo_accy, &rob_odo_acch, &rob_odo_encx, &rob_odo_ency, &rob_odo_ench);
//printf pour tester les valeurs
			printf("yo %f %f\n",rob_gpsy, loc[1]);
			
			wb_receiver_next_packet(receiver);
              	t+=TIME_STEP;	

		
	}
	return 0;
}
