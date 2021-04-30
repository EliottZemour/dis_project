#ifndef ODOMETRY_H
#define ODOMETRY_H 

void init_devices(int ts);
void controller_get_gps();
void controller_compute_mean_acc();
void controller_get_acc();
void controller_get_encoder();
void controller_init_log(const char* filename);
void controller_print_log(double time);

#endif