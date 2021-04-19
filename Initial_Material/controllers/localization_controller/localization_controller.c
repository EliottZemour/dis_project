#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "trajectories.h"
#include "odometry.h"

#define TIME_INIT_ACC 2


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


static measurement_t  _meas;
static pose_t  _pose, _odo_acc, _odo_enc;
static pose_t  _pose_origin = {-2.9, 0.0, 0.0};
static FILE *fp;


void init_devices(int ts);
void controller_compute_mean_acc();
void controller_get_acc();
void controller_get_encoder();
void controller_init_log(const char* filename);

/*
Initializes gps sensor, accelerometer sensor,
wheel encoders (position sensors), wheel motors.
int ts is taken as wb_robot_get_basic_time_step()
*/

void init_devices(int ts) {
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000);
  
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
  controller_init_log("localization.csv");
  
  while (wb_robot_step(time_step) != -1)  {
  // Use one of the two trajectories.
//    trajectory_1(dev_left_motor, dev_right_motor);
    trajectory_1(dev_left_motor, dev_right_motor);
    
    controller_get_acc();
    
    controller_get_encoder();
    
    if( wb_robot_get_time() < TIME_INIT_ACC )
      {
        controller_compute_mean_acc();
      }
    else
      {
        odo_compute_acc(&_odo_acc, _meas.acc, _meas.acc_mean);
        
        odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
      }
      
    controller_print_log(wb_robot_get_time());
  
  }

  
    return 0;
}

//====================================================
//====================================================
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

/**
 * @brief      Log the usefull informations about the simulation
 *
 * @param[in]  time  The time
 */
void controller_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            time, _pose.x, _pose.y , _pose.heading, _meas.gps[0], _meas.gps[1], 
      _meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
      _odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading);
  

}
}

void controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
  
    fprintf(fp, "time; pose_x; pose_y; pose_heading;  gps_x; gps_y; gps_z; acc_0; acc_1; acc_2; right_enc; left_enc; odo_acc_x; odo_acc_y; odo_acc_heading; odo_enc_x; odo_enc_y; odo_enc_heading; odo_enc_bonus_x; odo_enc_bonus_y; odo_enc_bonus_heading\n");
  


}

