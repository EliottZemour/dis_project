#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "odometry.h"

//----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC_BONUS false     	// Print odometry values computed with wheel encoders (Bonus)
#define VERBOSE_ODO_ENC false     			// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC false    			// Print odometry values computed with accelerometer
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 
 LE odo_compute_acc fonctionne mais il faut encore initialiser la position
 */
void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3])
{
	// Extraction of the heading from the wheel encoder method
	double a = _odo_pose_enc.heading;
	
            // Removal of bias from initialization
	double acc_wx = (acc[1]-acc_mean[1]);
	double acc_wy = -(acc[0]-acc_mean[0]);
	
/*
	double norm_acc = sqrt(pow(acc_wx,2.0)+pow(acc_wy,2.0));
	// Calculation of scalar acceleration
	double speed = norm_acc * _T;
	
	// Integration for speed determination and division according to axes
          _odo_speed_acc.x += speed * cos(a);
          _odo_speed_acc.y += speed * sin(a);
          
          double speedx = acc_wx * _T;
          double speedy = acc_wy * _T;
          static double x = 0;
          x+= speedx * _T;
          static double y = 0;
          y+=speedy * _T;
          
            
            // Integration for position determination
	_odo_pose_acc.x += _odo_speed_acc.x * _T ;
	_odo_pose_acc.y += _odo_speed_acc.y * _T ;
*/            

           double norm_acc = sqrt(pow(acc_wx,2.0)+pow(acc_wy,2.0));
	double dspeed = norm_acc * _T;
	
           double dvx = dspeed * cos(a);
           double dvy = dspeed * sin(a);
           
           //double dvx = acc_wx * _T;
           //double dvy = acc_wy * _T;
           
           _odo_speed_acc.x += dvx;
           _odo_speed_acc.y += dvy;
           
           double speed = sqrt(pow(_odo_speed_acc.x,2.0)+pow(_odo_speed_acc.y,2.0));
           double dl = speed * _T;
           
           double vx = speed * cos(a);
           double vy = speed * sin(a); 
           
           double dx__ = vx * _T;
           double dy__ = vy * _T;
           
           double dx_ = dl * cos(a);
           double dy_ = dl * sin(a);
           
           double dx = _odo_speed_acc.x * _T;
           double dy = _odo_speed_acc.y * _T;
           
           double cx = dx_/dx;
           double cy = dy_/dy;
           
           _odo_pose_acc.x += dx__;
           _odo_pose_acc.y += dy__;
           
            // Setting the heading of the acc method with the encoder value
           _odo_pose_acc.heading = a ;
            
	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
	
	
	printf("ODO with acceleration : \t x:%g\t y:%g\t h:%g\n", odo->x , odo->y , RAD2DEG(odo->heading));
	printf("ODO with acceleration_ :\t dx:%g\t dy:%g\t cx:%g\t cy:%g\n", dx , dy, cx, cy);
	printf("ODO with acceleration_ :\t dx_:%g\t dy_:%g\t dl:%g\n", dx_ , dy_, dl);
	printf("ODO with acceleration_ :\t dx__:%g\t dy__:%g\t dl:%g\n", dx__ , dy__, dl);
	printf("ODO with acceleration__ :\t vx:%g\t vy:%g\t v:%g\n", _odo_speed_acc.x , _odo_speed_acc.y, speed);
	printf("ODO with acceleration__ :\t vx_:%g\t vy_:%g\t v:%g\n", vx , vy, speed);
	printf("ODO with acceleration___ :\t dvx:%g\t dvy:%g\t dv:%g\n", dvx , dvy, dspeed);
	//printf("ODO with acceleration____ : %g %g\n", acc_wx, acc_wy);
	
	/*
 	printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));
 	printf("ODO with acceleration_ : %g %g %g %g %g\n", x , y , RAD2DEG(odo->heading), cos(a), sin(a));
 	printf("ODO with acceleration__ : %g %g %g %g\n", _odo_speed_acc.x, _odo_speed_acc.y, speed, _T);
 	printf("ODO with acceleration___ : %g %g %g %g\n", speedx, speedy, speed, _T);
           printf("ODO with acceleration____ : %g %g %g %g\n", acc_wx, acc_wy, norm_acc, _T);
           */
}


/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 LE odo_compute_acc fonctionne mais il faut encore initialiser la position
 */
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc)
{
	// Rad to meter
	Aleft_enc  *= WHEEL_RADIUS;

	Aright_enc *= WHEEL_RADIUS;

	// Compute forward speed and angular speed
	double omega = ( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * _T );

	double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * _T );

	// Apply rotation (Body to World)

	double a = _odo_pose_enc.heading;

	double speed_wx = speed * cos(a);

	double speed_wy = speed * sin(a);

	// Integration : Euler method
	_odo_pose_enc.x += speed_wx * _T;

	_odo_pose_enc.y += speed_wy * _T;

	_odo_pose_enc.heading += omega * _T;

	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

    	printf("ODO with wheel encoders :\t x:%g\t y:%g\t h:%g\n", odo->x , odo->y , RAD2DEG(odo->heading) );
    	printf("ODO with wheel encoders_ :\t vx:%g\t vy:%g\t v:%g\n", speed_wx, speed_wy , speed);
}



/**
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step)
{

 	memset(&_odo_pose_acc, 0 , sizeof(pose_t));

	memset(&_odo_speed_acc, 0 , sizeof(pose_t));

	memset(&_odo_pose_enc, 0 , sizeof(pose_t));


	_T = time_step / 1000.0;
}