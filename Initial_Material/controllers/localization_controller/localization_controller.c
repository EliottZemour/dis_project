#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/emitter.h>
#include "trajectories.h"
#include "odometry.h"
#include "localization_controller.h"
#include "kalman.h"
#define TIME_INIT_ACC 1


typedef struct 
{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
} measurement_t;

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;

static measurement_t  _meas;
static pose_t  _odo_acc, _odo_enc, _kalman_enc, _kalman_acc;
static FILE *fp;
static pose_t         _pose_origin = {-(2.9 - 0.12342), 0.0, 0.0};


void init_devices(int ts);
void controller_get_gps();
void controller_compute_mean_acc();
void controller_get_acc();
void controller_get_encoder();
void controller_init_log(const char* filename);
void controller_print_log(double time);
/*
Initializes gps sensor, accelerometer sensor,
wheel encoders (position sensors), wheel motors.
int ts is taken as wb_robot_get_basic_time_step()
*/

void init_devices(int ts) {
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps,1000);
  
  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);
  
  
  dev_left_encoder = wb_robot_get_device("left wheel sensor");
  dev_right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(dev_left_encoder,  ts);
  wb_position_sensor_enable(dev_right_encoder, ts);

  dev_left_motor = wb_robot_get_device("left wheel motor");
  dev_right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(dev_left_motor, INFINITY);
  wb_motor_set_position(dev_right_motor, INFINITY);
  wb_motor_set_velocity(dev_left_motor, 0.0);
  wb_motor_set_velocity(dev_right_motor, 0.0);
  
  controller_init_log("localization.csv");   // Initialization of the .csv file
  
  // Initialisation of the emitter node of the robot to send information to the supervisor
  emitter = wb_robot_get_device("emitter");
  if (emitter==0) printf("missing emitter\n");
}

//====================================================
//===================== MAIN =========================
//====================================================

int main() 
{
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);
  odo_reset(time_step);
  kalman_reset(time_step);
   
  char buffer[255]; // Buffer for emitter
  
  while (wb_robot_step(time_step) != -1)  {
  
            //    trajectory_2(dev_left_motor, dev_right_motor);
                   trajectory_1(dev_left_motor, dev_right_motor);

            // Functions to measure data from devices
            controller_get_gps();
    
            controller_get_acc();
    
            controller_get_encoder();
    
            if( wb_robot_get_time() < TIME_INIT_ACC )
                {
                  controller_compute_mean_acc();
                }
            else
                {
                   
                   // Computation of odometric coordinates at each time step, Function defined in odometry.c
                    
                   odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
                   odo_compute_acc(&_odo_acc, _meas.acc, _meas.acc_mean);
                   kalman_compute_enc(&_kalman_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc, _meas.gps);
                   kalman_compute_acc(&_kalman_acc, _meas.acc, _meas.acc_mean, _meas.gps);
       
                 }
                
                
                // Part dedicated to printing of values to be sent to the Supervisor
            sprintf(buffer,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",wb_robot_get_time(), _meas.gps[0], -_meas.gps[2], _meas.gps[1], 
            _odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading, _kalman_enc.x, _kalman_enc.y, _kalman_enc.heading,
            _kalman_acc.x, _kalman_acc.y, _kalman_acc.heading);
            wb_emitter_send(emitter,buffer,strlen(buffer));
            controller_print_log(wb_robot_get_time());

  
      }

  
    return 0;
}

//====================================================
//==================Get functions=====================
//====================================================

void controller_get_gps()
{

  // To Do : store the previous measurements of the gps (use memcpy)
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  // To Do : get the positions from webots for the gps. Uncomment and complete the following line Note : Use _robot.gps
  const double * gps_position = wb_gps_get_values(dev_gps);
  // To Do : Copy the gps_position into the measurment structure (use memcpy)
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));
  
  _meas.gps[0] -= _pose_origin.x;
  _meas.gps[1] -= _pose_origin.y;
  _meas.gps[2] -= _pose_origin.heading;

  //printf("ROBOT gps is at position: %g %g %g\n", _meas.gps[0], _meas.gps[1], _meas.gps[2]);
}


void controller_get_acc()
{
  // Call the function to get the accelerometer measurements.
  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  // Copy the acc_values into the measurment structure (use memcpy)
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));
  
  //printf("ROBOT acc : %g %g %g\n", _meas.acc[0], _meas.acc[1] , _meas.acc[2]);
}

void controller_get_encoder()
{
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  
  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;
  
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
  
  //printf("ROBOT enc : %g %g\n", _meas.left_enc, _meas.right_enc);
  
}

//====================================================
//=================Acc initialisation=================
//====================================================

void controller_compute_mean_acc()
{
  static int count = 0;
  
  count++;
  
  if( count > 20 ) // Remove the effects of strong acceleration at the begining
  {
    for(int i = 0; i < 3; i++)  
        _meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 1) + _meas.acc[i]) / (double) count;
  }
  
  if( count == (int) (TIME_INIT_ACC / (double) wb_robot_get_basic_time_step() * 1000) ) {
    printf("mean acc : %g %g %g\n",_meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2]);
    printf("Accelerometer initialization Done ! \n");
  }
}

//====================================================
//=================File related functions=============
//====================================================
/**
 * @brief      Log the useful informations about the simulation
 *
 * @param[in]  time  The time
 */
void controller_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g;  %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            time, _meas.gps[0], -_meas.gps[2], _meas.gps[1], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
      _odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading, _kalman_enc.x, _kalman_enc.y, _kalman_enc.heading,
      _kalman_acc.x, _kalman_acc.y, _kalman_acc.heading);
  

}

}

// Open the file and name the columns

void controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
  
    fprintf(fp, "time; gps_x; gps_y; gps_z; acc_0; acc_1; acc_2; right_enc; left_enc; odo_acc_x; odo_acc_y; odo_acc_heading; odo_enc_x; odo_enc_y; odo_enc_heading; kalman_enc_x; kalman_enc_y; kalman_enc_heading; kalman_acc_x; kalman_acc_y; kalman_acc_heading\n");
  


}