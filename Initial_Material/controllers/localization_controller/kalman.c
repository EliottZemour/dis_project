#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include <webots/robot.h> 

#include "linear_algebra.h"
#include "kalman.h"
#include "odometry.h"


//----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 	0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter

//-----------------------------------------------------------------------------------//
/*GLOBAL*/   
static double _T;
static pose_t _kalman_pose_acc, _kalman_pose_enc;

static double I[4][4] = {
              {1,  0,  0,  0},
              {0,  1,  0,  0},
              {0,  0,  1,  0},
              {0,  0,  0,  1}
            };  

static double A[4][4] = {
              {1,  0,  0,  0},
              {0,  1,  0,  0},
              {0,  0,  1,  0},
              {0,  0,  0,  1}
            };    

static double B_acc[4][2] ={
              { 0,  0},
              { 0,  0},        
              { 0,  0},
              { 0,  0}   
            }; 
            
            
static double B_enc[4][2] ={
              { 0,  0},
              { 0,  0},        
              { 0,  0},
              { 0,  0}   
            }; 

static double C[2][4] ={
                {1, 0, 0, 0},
                {0, 1, 0, 0}
              };
              
static double At[4][4];
      
static double Ct[4][2];

 //White noise
static double R[4][4] ={
              {0,   0,   0,   0},
              {0,   0,   0,   0},
              {0,   0,   0,   0},
              {0,   0,   0,   0}
            };
            
            
            
              
static double Q[2][2] ={
                {0.005, 0},
                {0, 0.005}
              }; 
  
          //Covariance matrix
static double Cov[4][4] ={
              {0.00001,  0,  0,  0},
              {0,  0.00001,  0,  0},
              {0,  0,  0.00001,  0},
              {0,  0,  0,  0.00001}
            };   
          
          //Kalman Matrix
static double K[4][2]={
              {0,0},
              {0,0},
              {0,0},
              {0,0},
            };
             
            
//-----------------------------------------------------------------------------------//
/*FUNCTIONS*/

/**
 * @brief      Reset the kalman filter to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void kalman_reset(int time_step)
{

 	memset(&_kalman_pose_acc, 0 , sizeof(pose_t));

	memset(&_kalman_pose_enc, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
	
	A[0][2] = _T;
	A[1][3] = _T;
	
	B_acc[2][0] = _T;
	B_acc[3][1] = _T;
	
	R[0][0] = 0.0005*_T;
	R[1][1] = 0.0005*_T;
	R[2][2] = 0.0005*_T;
	R[3][3] = 0.0005*_T;
	
	transpose(4,4,A,At);  
	transpose(2,4,C,Ct); 
}

/**
 * @brief      Compute the kalman filter using the acceleration and GPS
 *
 * @param      kalman    The kalman filtered value
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 * @param[in]  gps       The gps value
 
*/
void kalman_compute_acc(pose_t* kalman, const double acc[3], const double acc_mean[3], const double gps[3])
{
  double acc_wx = (acc[1] - acc_mean[1]);
  double acc_wy = -(acc[0] - acc_mean[0]);
  
  const int n = (int) (1/_T +1); // n = 62.5 but we take n = 63 because the gps value changes every 63 time step
  static int count = -1;
  count++;
  
  static double X[4][1] = {{0}, {0}, {0}, {0}}; //initial conditions
  double norm_acc = sqrt(pow(acc_wx,2.0)+pow(acc_wy,2.0));
  double a = _kalman_pose_enc.heading;
  double U[2][1] = {{norm_acc * cos(a)}, {norm_acc * sin(a)}};
  //double U[2][1] = {{acc_wx}, {acc_wy}};
  double Z[2][1] = {{gps[0]}, {-gps[2]}};
  
  double X_new[4][1];
  double Cov_new[4][4];  
  
  double aux41[4][1]; 
  double aux44[4][4]; 
  double aux42[4][2]; 
  double aux22[2][2]; 
  double aux21[2][1]; 
  
  //X_new  
  mult_mat(4,4,1,A,X,X_new);
  mult_mat(4,2,1,B_acc,U,aux41);
  
  double dspeed = sqrt(pow(aux41[2][0],2.0)+pow(aux41[3][0],2.0));
  
  add_mat(4,1,X_new,aux41,X_new);
  
  double speed = sqrt(pow(X_new[2][0],2.0)+pow(X_new[3][0],2.0));
  
  X_new[2][0] = speed * cos(a);
  X_new[3][0] = speed * sin(a);
    
  //Cov_new
  mult_mat(4,4,4,Cov, At, aux44);
  mult_mat(4,4,4,A, aux44, Cov_new);
   
  add_mat(4,4,Cov_new,R,Cov_new);
  
  if(count%n == 0){
    
    mult_mat(4,4,2,Cov_new,Ct,aux42);
    mult_mat(2,4,2,C, aux42, aux22);
   
    add_mat(2,2,aux22,Q,aux22);
    if(inv(2,aux22, aux22)){
    
      //Kalman matrix
      mult_mat(4,2,2,Ct,aux22,aux42);
      mult_mat(4,4,2,Cov_new, aux42, K);
      
      //X_new
      mult_mat(2,4,1,C,X_new,aux21);
      mult_scal(2,1,aux21,-1);
      add_mat(2,1,Z,aux21,aux21);
      
      mult_mat(4,2,1,K,aux21,aux41);
      
      add_mat(4,1,X_new, aux41, X_new);
      
      //Cov_new
      mult_mat(4,2,4,K,C,aux44);
      mult_scal(4,4,aux44,-1);
      add_mat(4,4,I,aux44,aux44);
      
      mult_mat(4,4,4,aux44,Cov_new,Cov_new);
      
      printf("GPS : %g %g\n", Z[0][0] , Z[1][0]);
    }
  }
  
  for(int i=0;i<4;++i){
    X[i][0]=X_new[i][0];
  }
  
  // Cov=Cov_new  MISSING!!!
  for (int i=0; i<4; ++i){
    for (int j= 0; j<4; ++j){    
      Cov[i][j] = Cov_new[i][j];
    }
  }
  
  _kalman_pose_acc.x = X_new[0][0];
  _kalman_pose_acc.y = X_new[1][0];
            
  _kalman_pose_acc.heading = a;
  
  memcpy(kalman, &_kalman_pose_acc, sizeof(pose_t));
	
	
  double speed_ = sqrt(pow(X[2][0],2.0)+pow(X[3][0],2.0));
  
  printf("Kalman with acceleration :\t x:%g\t y:%g\t h:%g\n", kalman->x , kalman->y , RAD2DEG(kalman->heading));

  printf("Kalman with acceleration__ :\t vx:%g\t vy:%g\t v:%g\n", X[2][0] , X[3][0], speed_);
  printf("Kalman with acceleration_ :\t dvx:%g\t dvy:%g\t dv:%g\n", aux41[2][0] , aux41[3][0], dspeed);
  printf("GPS : %g %g\n \n", gps[0], -gps[2]);  
} 

/**
 * @brief      Compute the kalman filter using the encoder and GPS
 *
 * @param      kalman    The kalman filtered value
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 * @param[in]  gps       The gps value
 
*/
void kalman_compute_enc(pose_t* kalman, double Aleft_enc, double Aright_enc, const double gps[3])
{
  // Rad to meter
  Aleft_enc  *= WHEEL_RADIUS;
  Aright_enc *= WHEEL_RADIUS;

  const int n = (int) (1/_T +1); // n = 62.5 but we take n = 63 because the gps value changes every 63 time step
  static int count = -1;
  count++;
  
  static double X[4][1] = {{0}, {0}, {0}, {0}}; //initial conditions
  double U[2][1] = {{Aleft_enc}, {Aright_enc}};
  double Z[2][1] = {{gps[0]}, {-gps[2]}};
  
  double X_new[4][1];
  double Cov_new[4][4];  
  
  double aux41[4][1]; 
  double aux44[4][4]; 
  double aux42[4][2]; 
  double aux22[2][2]; 
  double aux21[2][1]; 
  
  double a = _kalman_pose_enc.heading;
  B_enc[2][0] = cos(a)/(2.0*_T);
  B_enc[2][1] = cos(a)/(2.0*_T);
  B_enc[3][0] = sin(a)/(2.0*_T);
  B_enc[3][1] = sin(a)/(2.0*_T);
  
  //X_new  
  mult_mat(4,4,1,A,X,X_new);
  mult_mat(4,2,1,B_enc,U,aux41);
  mult_scal(4,1,aux41,_T);
  add_mat(4,1,X_new,aux41,X_new);
  
  double speed = sqrt(pow(X_new[2][0],2.0)+pow(X_new[3][0],2.0));
  
  X_new[2][0] = speed * cos(a);
  X_new[3][0] = speed * sin(a);
    
  //Cov_new
  mult_mat(4,4,4,Cov, At, aux44);
  mult_mat(4,4,4,A, aux44, Cov_new);
   
  add_mat(4,4,Cov_new,R,Cov_new);
  
  if(count%n == 0){
    
    mult_mat(4,4,2,Cov_new,Ct,aux42);
    mult_mat(2,4,2,C, aux42, aux22);
   
    add_mat(2,2,aux22,Q,aux22);
    if(inv(2,aux22, aux22)){
    
      //Kalman matrix
      mult_mat(4,2,2,Ct,aux22,aux42);
      mult_mat(4,4,2,Cov_new, aux42, K);
      
      //X_new
      mult_mat(2,4,1,C,X_new,aux21);
      mult_scal(2,1,aux21,-1);
      add_mat(2,1,Z,aux21,aux21);
      
      mult_mat(4,2,1,K,aux21,aux41);
      
      add_mat(4,1,X_new, aux41, X_new);
      
      //Cov_new
      mult_mat(4,2,4,K,C,aux44);
      mult_scal(4,4,aux44,-1);
      add_mat(4,4,I,aux44,aux44);
      
      mult_mat(4,4,4,aux44,Cov_new,Cov_new);
    }
  }
  
  for(int i=0;i<4;++i){
    X[i][0]=X_new[i][0];
  }
  
  _kalman_pose_enc.x = X_new[0][0];
  _kalman_pose_enc.y = X_new[1][0];
  
  double omega = ( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * _T );
  _kalman_pose_enc.heading += omega * _T;
  //_kalman_pose_enc.heading = atan2(X_new[2][0], X_new[3][0]);
  
  memcpy(kalman, &_kalman_pose_enc, sizeof(pose_t));
	
  printf("Kalman with encoder : %g %g %g\n", kalman->x , kalman->y , RAD2DEG(kalman->heading));  
  printf("Time managment : %g %d %d\n",wb_robot_get_time(), count, n);  
 } 