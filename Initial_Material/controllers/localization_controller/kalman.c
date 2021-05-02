#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "odometry.h"
#include "kalman.h"
#include "linear_algebra.h"

//----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 	0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter

//-----------------------------------------------------------------------------------//
/*GLOBAL*/   
static double _T;
static pose_t _kalman_pose_acc, _kalman_pose_enc;

const double I[4][4] ={
    {1,  0,  0,  0},
    {0,  1,  0,  0},
    {0,  0,  1,  0},
    {0,  0,  0,  1}
  };  

const double A[4][4] ={
    {1,  0, _T,  0},
    {0,  1,  0, _T},
    {0,  0,  1,  0},
    {0,  0,  0,  1}
  };    

const double B_acc[4][2] ={
    { 0,  0},
    { 0,  0},        
    {_T,  0},
    { 0, _T}   
  }; 
  
static double B_enc[4][2]; 

const double C[2][4] ={
    {1, 0, 0, 0},
    {0, 1, 0, 0}
  };
  
static double At[4][4];
transpose(4,4,A,At);  

static double Ct[4][2];
transpose(2,4,C,Ct); 


//White noise
const double R[4][4] ={
    {0.0005*_T,   0,   0,   0},
    {0,   0.0005*_T,   0,   0},
    {0,   0,   0.0001*_T,   0},
    {0,   0,   0,   0.0001*_T}
  };
  
const double Q[2][2] ={
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
  }, 

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
  
  double t = wb_robot_get_time();
  const int n = (int) (1/_T);
  int count = (int) (t/_T);
  
  static double X[4][1] = {{0}, {0}, {0}, {0}}; //initial conditions
  double U[2][1] = {{acc_wx}, {acc_wy}};
  double Z[2][1] = {{gps[0]}, -{gps[2]}};
  
  double X_new[4][1];
  double Cov_new[4][4];  
  
  double aux41[4][1]; 
  double aux44[4][4]; 
  double aux42[4][2]; 
  double aux22[2][2]; 
  double aux21[2][1]; 
  
  //X_new  
  mutl_mat(4,4,1,A,X,X_new);
  mult_mat(4,2,1,B_acc,U,aux41);
  add_mat(4,1,X_new,aux41,X_new);
    
  //Cov_new
  mutl_mat(4,4,4,Cov, At, aux44);
  mult_mat(4,4,4,A, aux44, Cov_new);
   
  add_mat(4,4,Cov_new,R,Cov_new);
  
  
  if(count%n == 0){
    mutl_mat(4,4,2,Cov_new,Ct,aux42);
    mult_mat(2,4,2,C, aux42, aux22);
   
    add_mat(2,2,aux22,Q,aux22);
    if(inv(2,aux22, aux22)){
    
      //Kalman matrix
      mutl_mat(4,2,2,Ct,aux22,aux42);
      mult_mat(4,4,2,Cov_new, aux42, K);
      
      //X_new
      mult_mat(2,4,1,C,X_new,aux21);
      mult_scal(2,1,aux21,-1);
      add_mat(2,1,Z,aux21,aux21);
      
      mult_mat(4,2,1,K,aux21,aux41);
      
      add_mat(4,1,X_new, aux41, X_new);
      
      //Cov_new
      mult_mat(4,2,4,K,C,aux44);
      mult_sacl(4,4,aux44,-1);
      add_mult(4,4,I,aux44,aux44);
      
      mult_mat(4,4,4,aux44,Cov_new,Cov_new);
    }
  }
  
  for(int i=0;i<4;++i){
    X[i][0]=X_new[i][0];
  }
  
  _kalman_pose_acc.x = X_new[0][0];
  _kalman_pose_acc.y = X_new[1][0];
            
  _kalman_pose_acc.heading = atan2(X_new[2][0], X_new[3][0]);
  
  memcpy(kalman, &_kalman_pose_acc, sizeof(pose_t));
	
  //printf("Kalman with acceleration : %g %g %g\n", kalman->x , kalman->y , RAD2DEG(kalman->heading));  

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

  const int n = (int) (1/_T);
  double t = wb_robot_get_time();
  int count = (int) (t/_T);
  
  static double X[4][1] = {{0}, {0}, {0}, {0}}; //initial conditions
  double U[2][1] = {{Aleft_enc}, {Aright_enc}};
  double Z[2][1] = {{gps[0]}, -{gps[2]}};
  
  double X_new[4][1];
  double Cov_new[4][4];  
  
  double aux41[4][1]; 
  double aux44[4][4]; 
  double aux42[4][2]; 
  double aux22[2][2]; 
  double aux21[2][1]; 
  
  double a = _kalman_pose_enc.heading;
  B_enc = {
    {        0,          0},
    {        0,          0},
    {cos(a)/(2.0*_T), cos(a)/(2.0*_T)},
    {sin(a)/(2.0*_T), sin(a)/(2.0*_T)}
  };
  
  //X_new  
  mutl_mat(4,4,1,A,X,X_new);
  mult_mat(4,2,1,B_enc,U,aux41);
  add_mat(4,1,X_new,aux41,X_new);
    
  //Cov_new
  mutl_mat(4,4,4,Cov, At, aux44);
  mult_mat(4,4,4,A, aux44, Cov_new);
   
  add_mat(4,4,Cov_new,R,Cov_new);
  
  
  if(count%n == 0){
    mutl_mat(4,4,2,Cov_new,Ct,aux42);
    mult_mat(2,4,2,C, aux42, aux22);
   
    add_mat(2,2,aux22,Q,aux22);
    if(inv(2,aux22, aux22)){
    
      //Kalman matrix
      mutl_mat(4,2,2,Ct,aux22,aux42);
      mult_mat(4,4,2,Cov_new, aux42, K);
      
      //X_new
      mult_mat(2,4,1,C,X_new,aux21);
      mult_scal(2,1,aux21,-1);
      add_mat(2,1,Z,aux21,aux21);
      
      mult_mat(4,2,1,K,aux21,aux41);
      
      add_mat(4,1,X_new, aux41, X_new);
      
      //Cov_new
      mult_mat(4,2,4,K,C,aux44);
      mult_sacl(4,4,aux44,-1);
      add_mult(4,4,I,aux44,aux44);
      
      mult_mat(4,4,4,aux44,Cov_new,Cov_new);
    }
  }
  
  for(int i=0;i<4;++i){
    X[i][0]=X_new[i][0];
  }
  
  _kalman_pose_enc.x = X_new[0][0];
  _kalman_pose_enc.y = X_new[1][0];
            
  _kalman_pose_enc.heading = atan2(X_new[2][0], X_new[3][0]);
  
  memcpy(kalman, &_kalman_pose_enc, sizeof(pose_t));
	
  //printf("Kalman with encoder : %g %g %g\n", kalman->x , kalman->y , RAD2DEG(kalman->heading));  

} 