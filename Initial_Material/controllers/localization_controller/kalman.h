#ifndef ODOMETRY_H
#define ODOMETRY_H 

#define RAD2DEG(X)      X / M_PI * 180.0

void kalman_compute_acc(pose_t* kalman, const double acc[3], const double acc_mean[3], const double gps[3]);
void kalman_compute_enc(pose_t* kalman, double Aleft_enc, double Aright_enc, const double gps[3])
void kalman_reset(int time_step);

#endif