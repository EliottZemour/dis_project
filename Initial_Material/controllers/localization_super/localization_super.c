
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
float metric_gps =0;
float metric_acc = 0;
float metric_enc = 0;
float metric_kalman_acc = 0;
float metric_kalman_enc = 0;

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

// Initialization of the log file

void controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
   fprintf(fp, "time; loc_x; loc_z_; theta; metric_gps; metric_acc; metric_enc ; metric_kalman_acc; metric_kalman_enc\n");
  


}

// printing of data in the log file "pose.csv"
void controller_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g;  %g; %g; %g; %g; %g; %g\n",
            time, loc[0],loc[1], RAD2DEG(loc[2]), metric_gps, metric_acc, metric_enc, metric_kalman_acc, metric_kalman_enc);
  

}
}





/*
 * Main function.
 */
 	float fit_acc =0;
int main() 
{	float index = 0.0;
           float index_gps=0.0;
	char *inbuffer;
           float rob_time, rob_gpsx, rob_gpsy, rob_gpsz, rob_odo_accx,rob_odo_accy, rob_odo_acch, rob_odo_encx, rob_odo_ency, rob_odo_ench;
           float rob_kalman_encx, rob_kalman_ency, rob_kalman_ench, rob_kalman_accx, rob_kalman_accy, rob_kalman_acch;
	controller_init_log("pose.csv");
	reset();
	for(;;) {
          	     wb_robot_step(TIME_STEP);
		
		if (t % 10 == 0) {

			// Get data from Scene Tree
			loc[0] = wb_supervisor_field_get_sf_vec3f(robs_trans[0])[0]; // X
			loc[1] = wb_supervisor_field_get_sf_vec3f(robs_trans[0])[2]; // Y
			loc[2] = wb_supervisor_field_get_sf_rotation(robs_rotation[0])[3]; // THETA	
		}
		// Get data from robot in order to calculate the metrics
		inbuffer = (char*) wb_receiver_get_data(receiver);
                                sscanf(inbuffer,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",&rob_time, &rob_gpsx,&rob_gpsy, &rob_gpsz, &rob_odo_accx, &rob_odo_accy, &rob_odo_acch, &rob_odo_encx, &rob_odo_ency, &rob_odo_ench, &rob_kalman_encx, &rob_kalman_ency, &rob_kalman_ench, &rob_kalman_accx, &rob_kalman_accy, &rob_kalman_acch);			wb_receiver_next_packet(receiver);
			
		
		// Calculation of the metrics, the GPS takes 1 second to initialize, so its first second is skipped
              	
              	if(t>=1000){
                      metric_gps += sqrt(pow(loc[0]-rob_gpsx+2.78,2.0)+pow(loc[1]+rob_gpsy,2.0));
                      index_gps += 1.0;
                      }
                      metric_acc += sqrt(pow(loc[0]-rob_odo_accx+2.9,2.0)+pow(loc[1]-rob_odo_accy,2.0));
                      metric_enc += sqrt(pow(loc[0]-rob_odo_encx+2.9,2.0)+pow(loc[1]+rob_odo_ency,2.0));
                      metric_kalman_acc += sqrt(pow(loc[0]-rob_kalman_accx+2.9,2.0)+pow(loc[1]-rob_kalman_accy,2.0));
                      metric_kalman_enc += sqrt(pow(loc[0]-rob_kalman_encx+2.9,2.0)+pow(loc[1]+rob_kalman_ency,2.0));
                      	index += 1.0;
                      	
                      
                      // Division by the number of steps so far in the simulation to determine the average error
                      	
                      	metric_gps = metric_gps / index_gps;
                      	metric_acc = metric_acc / index;
                      	metric_enc = metric_enc / index;
                      	metric_kalman_acc = metric_kalman_acc / index;
                      	metric_kalman_enc = metric_kalman_enc / index;                      	
                      	
                      	// writing in the csv file
		controller_print_log(t/1000.0);		
              	t+=TIME_STEP;	
	}
	
}
