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

/*VERBOSE_FLAGS*/
#define VERBOSE_KALMAN_ENC       false     	// Print Kalman values computed with wheel encoders
#define VERBOSE_KALMAN_ACC       false    	// Print Kalman values computed with accelerometer

//-----------------------------------------------------------------------------------//
/*GLOBAL*/   
static double _T;
static pose_t _kalman_pose_acc, _kalman_pose_enc;

// Identity matrix
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
static double At[4][4];


static double C[2][4] = { 
                {1, 0, 0, 0},
                {0, 1, 0, 0}
              };      
static double Ct[4][2];


static double B_acc[4][2] = {
                  {0, 0},
                  {0, 0},        
                  {0, 0},
                  {0, 0}   
            };             
            
static double B_enc[4][2] = {
                  {0, 0},
                  {0, 0},        
                  {0, 0},
                  {0, 0}   
            }; 


//White noise
static double R[4][4] = { 
              {0,   0,   0,   0},
              {0,   0,   0,   0},
              {0,   0,   0,   0},
              {0,   0,   0,   0}
            };         
              
static double Q[2][2] ={
                {0.05, 0},
                {0, 0.05}
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
	
	// _T dependencies
	A[0][2] = _T;
	A[1][3] = _T;
	
	B_acc[2][0] = _T;
	B_acc[3][1] = _T;
	
	R[0][0] = 0.05*_T;
	R[1][1] = 0.05*_T;
	R[2][2] = 0.05*_T;
	R[3][3] = 0.05*_T;
	
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
        //-----------------------------------------------------------------------------------//
        /*INITIAL CONDITIONS*/
        
        // The two matrix X and Cov are defined localy so that there are not ichanged by the other Kalman function
        // Position-velocity matrix
        static double X[4][1] = {{0}, {0}, {0}, {0}}; 
           
        // Covariance matrix
        static double Cov[4][4] ={
                      {0.001,  0,  0,  0},
                      {0,  0.001,  0,  0},
                      {0,  0,  0.001,  0},
                      {0,  0,  0,  0.001}
                    };   
                    
        //-----------------------------------------------------------------------------------//
        /*TIME MANAGEMENT FOR GPS UPDATE*/            
     
        const int n = (int) (1/_T +1); // n = 62.5 but we take n = 63 because the gps value changes every 63 time step
        static int count = -1;
        count++;
        
        //----------------------------------------------------------------------------------//
        /*VARIABLES*/
        
        // Extraction of the heading from the wheel encoder method
        double a = _kalman_pose_enc.heading;
        
        // Removal of bias from initialization
        double acc_wx = (acc[1] - acc_mean[1]);
        double acc_wy = -(acc[0] - acc_mean[0]); 
        
        // Input value for the Kalman filter
        double U[2][1] = {{acc_wx}, {acc_wy}};
        double Z[2][1] = {{gps[0]}, {-gps[2]}};
        
        // Temp matrix
        double X_new[4][1];
        double Cov_new[4][4];  
        
        double aux41[4][1]; 
        double aux44[4][4]; 
        double aux42[4][2]; 
        double aux22[2][2]; 
        double aux21[2][1]; 

        //----------------------------------------------------------------------------------//
        /*ALGORITHM*/
        
        //X_new  
        mult_mat(4,4,1,A,X,X_new);
        mult_mat(4,2,1,B_acc,U,aux41);        
        add_mat(4,1,X_new,aux41,X_new);
        
        double speed = sqrt(pow(X_new[2][0],2.0)+pow(X_new[3][0],2.0));
        
        X_new[2][0] = speed * cos(a);
        X_new[3][0] = speed * sin(a);
        
        //printM(4,1,X_new);
          
        //Cov_new
        mult_mat(4,4,4,Cov, At, aux44);        
        mult_mat(4,4,4,A, aux44, Cov_new);
        add_mat(4,4,Cov_new,R,Cov_new);
        
        //printM(4,4,Cov_new);
        
        if(count%n == 0){
              
              mult_mat(4,4,2,Cov_new,Ct,aux42);
              mult_mat(2,4,2,C, aux42, aux22);
              add_mat(2,2,aux22,Q,aux22);  
              
              if(inv(2,aux22, aux22)){ // aux22 is now its inverse 
                                    
                    //Kalman matrix
                    mult_mat(4,2,2,Ct,aux22,aux42);
                    mult_mat(4,4,2,Cov_new, aux42, K);
                    
                    //printM(4,2,K);
                    
                    //X_new
                    mult_mat(2,4,1,C,X_new,aux21);
                    mult_scal(2,1,aux21,-1);
                    add_mat(2,1,Z,aux21,aux21);
                    
                    mult_mat(4,2,1,K,aux21,aux41);
                    add_mat(4,1,X_new, aux41, X_new);
                    
                    //printM(4,1,X_new);
                    
                    //Cov_new
                    mult_mat(4,2,4,K,C,aux44);
                    mult_scal(4,4,aux44,-1);
                    add_mat(4,4,I,aux44,aux44);
                    
                    mult_mat(4,4,4,aux44,Cov_new,Cov_new);
                    
                    //printM(4,4,Cov_new);
                    
                    //printf("GPS : x:%g y:%g\n", Z[0][0] , Z[1][0]);
                    
              }
        }
        
        
        // X=X_new 
        for(int i=0;i<4;++i){
              X[i][0]=X_new[i][0];
        }
        
        // Cov=Cov_new  
        for (int i=0; i<4; ++i){
              for (int j= 0; j<4; ++j){    
                    Cov[i][j] = Cov_new[i][j];
              }
        }
        
        // Update the variable
        _kalman_pose_acc.x = X_new[0][0];
        _kalman_pose_acc.y = X_new[1][0];           
        _kalman_pose_acc.heading = a;
        
        memcpy(kalman, &_kalman_pose_acc, sizeof(pose_t));        
        
        if (VERBOSE_KALMAN_ACC){
            printf("Kalman with acceleration :\t x:%g\t y:%g\t h:%g\n", kalman->x , kalman->y , RAD2DEG(kalman->heading));
            printf("Kalman with acceleration :\t vx:%g\t vy:%g\t v:%g\n", X[2][0] , X[3][0], speed);
        }
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
        //-----------------------------------------------------------------------------------//
        /*INITIAL CONDITIONS*/
        
        // The two matrix X and Cov are defined localy so that there are not ichanged by the other Kalman function
        // Position-velocity matrix
        static double X[4][1] = {{0}, {0}, {0}, {0}}; 
           
        // Covariance matrix
        static double Cov[4][4] ={
                      {0.001,  0,  0,  0},
                      {0,  0.001,  0,  0},
                      {0,  0,  0.001,  0},
                      {0,  0,  0,  0.001}
                    };        
        
        //-----------------------------------------------------------------------------------//
        /*TIME MANAGEMENT FOR GPS UPDATE*/   
        
        const int n = (int) (1/_T +1); // n = 62.5 but we take n = 63 because the gps value changes every 63 time step
        static int count = -1;
        count++;
 
        //----------------------------------------------------------------------------------//
        /*VARIABLES*/
        
        // Angle dependencies
        double a = _kalman_pose_enc.heading;
        B_enc[2][0] = cos(a)/(2.0*_T);
        B_enc[2][1] = cos(a)/(2.0*_T);
        B_enc[3][0] = sin(a)/(2.0*_T);
        B_enc[3][1] = sin(a)/(2.0*_T);
                  
        // Rad to meter
        Aleft_enc  *= WHEEL_RADIUS;
        Aright_enc *= WHEEL_RADIUS;
        
        // Input value for the Kalman filter
        double U[2][1] = {{Aleft_enc}, {Aright_enc}};
        double Z[2][1] = {{gps[0]}, {-gps[2]}};
        
        // Temp matrix
        double X_new[4][1];
        double Cov_new[4][4];  
        
        double aux41[4][1]; 
        double aux44[4][4]; 
        double aux42[4][2]; 
        double aux22[2][2]; 
        double aux21[2][1]; 
        
        //----------------------------------------------------------------------------------//
        /*ALGORITHM*/
        
        //X_new  
        mult_mat(4,4,1,A,X,X_new);
        mult_mat(4,2,1,B_enc,U,aux41);
        mult_scal(4,1,aux41,_T);
        add_mat(4,1,X_new,aux41,X_new);
        
        double speed = sqrt(pow(X_new[2][0],2.0)+pow(X_new[3][0],2.0));
        
        X_new[2][0] = speed * cos(a);
        X_new[3][0] = speed * sin(a);
        
        //printM(4,1,X_new);
          
        //Cov_new
        mult_mat(4,4,4,Cov, At, aux44);
        mult_mat(4,4,4,A, aux44, Cov_new);
        add_mat(4,4,Cov_new,R,Cov_new);
        
        //printM(4,4,Cov_new);
        
        if(count%n == 0){
          
              mult_mat(4,4,2,Cov_new,Ct,aux42);
              mult_mat(2,4,2,C, aux42, aux22);
              add_mat(2,2,aux22,Q,aux22);
              
              if(inv(2,aux22, aux22)){ // aux22 is now its inverse 
                  
                    //Kalman matrix
                    mult_mat(4,2,2,Ct,aux22,aux42);
                    mult_mat(4,4,2,Cov_new, aux42, K);
                    
                    //printM(4,2,K);
                    
                    //X_new
                    mult_mat(2,4,1,C,X_new,aux21);
                    mult_scal(2,1,aux21,-1);
                    add_mat(2,1,Z,aux21,aux21);
                    
                    mult_mat(4,2,1,K,aux21,aux41);
                    add_mat(4,1,X_new, aux41, X_new);
                    
                    //printM(4,1,X_new);
                    
                    //Cov_new
                    mult_mat(4,2,4,K,C,aux44);
                    mult_scal(4,4,aux44,-1);
                    add_mat(4,4,I,aux44,aux44);
                    
                    mult_mat(4,4,4,aux44,Cov_new,Cov_new);
                    
                    //printM(4,4,Cov_new);
                    
                    //printf("GPS : x:%g y:%g\n", Z[0][0] , Z[1][0]);
              }
        }
        
        
        // X=X_new 
        for(int i=0;i<4;++i){
          X[i][0]=X_new[i][0];
        }
        // Cov=Cov_new  MISSING!!!
        for (int i=0; i<4; ++i){
          for (int j= 0; j<4; ++j){    
            Cov[i][j] = Cov_new[i][j];
          }
        }
        
        
        // Update the variable
        _kalman_pose_enc.x = X_new[0][0];
        _kalman_pose_enc.y = X_new[1][0];
        
        double omega = ( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * _T );
        _kalman_pose_enc.heading += omega * _T;
        
        memcpy(kalman, &_kalman_pose_enc, sizeof(pose_t));
      	
      	
        if (VERBOSE_KALMAN_ENC){
            printf("Kalman with encoder : %g %g %g\n", kalman->x , kalman->y , RAD2DEG(kalman->heading));  
            printf("Kalman with acceleration :\t vx:%g\t vy:%g\t v:%g\n", X[2][0] , X[3][0], speed);
        }
 } 