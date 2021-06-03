%% Part A : Plot accelerometers values 

[N_SIM, T_SIM, T, data] = read_log();

f = figure('Name','accelerometer [m/s^2]');

% Plot x acceleration 
subplot(3,1,1);
plot(data.time , data.acc(:,1));
title('acc[0]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min( data.acc(:,1)),max(data.acc(:,1))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y acceleration
subplot(3,1,2);
plot(data.time , data.acc(:,2));
title('acc[1]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min(data.acc(:,2)),max(data.acc(:,2))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot z acceleration
subplot(3,1,3);
plot(data.time , data.acc(:,3));
title('acc[2]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min(data.acc(:,3)),max(data.acc(:,3))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part B : Plot the odometry computed using the accelerometer on Webots vs time

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2] vs time [s]');

% Plot x : odometry vs ground truth (gps)
subplot(2,1,1);
plot(data.time, data.gps_x, '.'); hold on;
plot(data.time, data.odo_acc_x, '.');
title('x trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([data.odo_acc_x;  data.gps_x]),max([data.odo_acc_x;  data.gps_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y : odometry vs ground truth (gps)
subplot(2,1,2);
plot(data.time, data.gps_y, '.'); hold on;
plot(data.time, data.odo_acc_y, '.');
title('y trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('Time [s]'); ylabel('y [m]');
y_lim = [min([data.odo_acc_y;  data.gps_y]),max([data.odo_acc_y;  data.gps_y])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part C : Plot the odometry computed using the accelerometer on Webots

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(data.gps_x(3:end) , data.gps_y(3:end)); hold on;
plot(data.odo_acc_x, data.odo_acc_y, '.');
title('trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('x [m]'); ylabel('y [m]');
y_lim = [min([data.odo_acc_y;  data.odo_acc_x]),max([data.odo_acc_y;  data.odo_acc_x])];
axis equal;

%% Part D : Plot the odometry computed using the wheel encoders on Webots vs time

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the wheel encoders
f = figure('Name','Webots : Odometry using wheel encoders [m/s^2] vs time [s]');

% Plot x : odometry vs ground truth (gps)
subplot(2,1,1);
plot(data.time, data.gps_x, '.'); hold on;
plot(data.time, data.odo_enc_x, '.');
title('x trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : wheel encoders');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([data.odo_enc_x;  data.gps_x]),max([data.odo_enc_x;  data.gps_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y : odometry vs ground truth (gps)
subplot(2,1,2);
plot(data.time, data.gps_y, '.'); hold on;
plot(data.time, data.odo_enc_y, '.');
title('y trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : wheel encoders');
xlabel('Time [s]'); ylabel('y [m]');
y_lim = [min([data.odo_enc_y;  data.gps_y]),max([data.odo_enc_y;  data.gps_y])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part E : Plot the odometry computed using the wheel encoders on Webots

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the wheel encoders
f = figure('Name','Webots : Odometry using wheel encoders [Rad]'); 

% Plot x -y plan : odometry vs ground truth (gps)
plot(data.gps_x(3:end) , data.gps_y(3:end)); hold on;
plot(data.odo_enc_x , data.odo_enc_y);
title('x -y plan : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data.odo_enc_x;  data.gps_x]),max([data.odo_enc_x;  data.gps_x])];
y_lim = [min([data.odo_enc_y;  data.gps_y]),max([data.odo_enc_y;  data.gps_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

%% Part F : Plot the Kalman filter computed using the wheel encoders on Webots vs time

[N_SIM, T_SIM, T, data] = read_log();

% Plot the Kalman filter computed using the wheel encoders
f = figure('Name','Webots : Kalman using wheel encoders [Rad] vs time [s]'); 

% Plot x : Kalman filter vs ground truth (gps)
subplot(2,1,1);
plot(data.time, data.gps_x, '.'); hold on;
plot(data.time, data.kalman_enc_x, '.');
title('x trajectory : Kalman filter vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Kalman : Wheel encoders');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([data.kalman_enc_x;  data.gps_x]),max([data.kalman_enc_x;  data.gps_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y : Kalman filter vs ground truth (gps)
subplot(2,1,2);
plot(data.time, data.gps_y, '.'); hold on;
plot(data.time, data.kalman_enc_y, '.');
title('y trajectory : Kalman filter vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Kalman : Wheel encoders');
xlabel('Time [s]'); ylabel('y [m]');
y_lim = [min([data.kalman_enc_y;  data.gps_y]),max([data.kalman_enc_y;  data.gps_y])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part G : Plot the Kalman filter computed using the wheel encoders on Webots 

[N_SIM, T_SIM, T, data] = read_log();

% Plot the Kalman filter computed using the wheel encoders
f = figure('Name','Webots : Kalman using wheel encoders [Rad]'); 

% Plot x -y plan : Kalman filter vs ground truth (gps)
plot(data.gps_x(3:end) , data.gps_y(3:end), '-'); hold on;
plot(data.kalman_enc_x , data.kalman_enc_y, '.');
title('x -y plan : kalman filter for wheel encoder vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Kalman : Wheel encoders');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data.kalman_enc_x;  data.gps_x]),max([data.kalman_enc_x;  data.gps_x])];
y_lim = [min([data.kalman_enc_y;  data.gps_y]),max([data.kalman_enc_y;  data.gps_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

%% Part H : Plot the Kalman filter computed using accelerometer on Webots vs time

[N_SIM, T_SIM, T, data] = read_log();

% Plot the Kalman filter computed using the accelerometer
f = figure('Name','Webots : Kalman using accelerometer [m/s^2] vs time [s]'); 

% Plot x : Kalman filter vs ground truth (gps)
subplot(2,1,1);
plot(data.time, data.gps_x, '.'); hold on;
plot(data.time, data.kalman_acc_x, '.');
title('x trajectory : Kalman filter vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Kalman : Accelerometer');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([data.kalman_acc_x;  data.gps_x]),max([data.kalman_acc_x;  data.gps_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y : Kalman filter vs ground truth (gps)
subplot(2,1,2);
plot(data.time, data.gps_y, '.'); hold on;
plot(data.time, data.kalman_acc_y, '.');
title('y trajectory : Kalman filter vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Kalman : Accelerometer');
xlabel('Time [s]'); ylabel('y [m]');
y_lim = [min([data.kalman_acc_y;  data.gps_y]),max([data.kalman_acc_y;  data.gps_y])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part I : Plot the Kalman filter computed using accelerometer on Webots 

[N_SIM, T_SIM, T, data] = read_log();

% Plot the Kalman filter computed using the accelerometer
f = figure('Name','Webots : Kalman using accelerometer [m/s^2]]'); 

% Plot x -y plan : Kalman filter vs ground truth (gps)
plot(data.gps_x(3:end) , data.gps_y(3:end), '-'); hold on;
plot(data.kalman_acc_x , data.kalman_acc_y, '.');
title('x -y plan : kalman filter for accelerometer vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Kalman : Accelerometer');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data.kalman_acc_x;  data.gps_x]),max([data.kalman_acc_x;  data.gps_x])];
y_lim = [min([data.kalman_acc_y;  data.gps_y]),max([data.kalman_acc_y;  data.gps_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

%% Part J : TEST : traj vs time

[N_SIM, T_SIM, T, data] = read_log();

f = figure('Name','TEST : traj vs time'); 

% Plot x :
subplot(2,1,1);
plot(data.time, data.odo_acc_x, '.'); hold on;
plot(data.time, data.kalman_acc_x, '.');
title('x trajectory : Kalman filter vs odometry');
legend('Odometry : Accelerometer', 'Kalman : Accelerometer');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([data.kalman_acc_x;  data.odo_acc_x]),max([data.kalman_acc_x;  data.odo_acc_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y : 
subplot(2,1,2);
plot(data.time, data.odo_acc_y, '.');; hold on;
plot(data.time, data.kalman_acc_y, '.');
title('y trajectory : Kalman filter vs odometry');
legend('Odometry : Accelerometer', 'Kalman : Accelerometer');
xlabel('Time [s]'); ylabel('y [m]');
y_lim = [min([data.kalman_acc_y;  data.odo_acc_y]),max([data.kalman_acc_y;  data.odo_acc_y])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part K : TEST : 2D traj

[N_SIM, T_SIM, T, data] = read_log();

f = figure('Name','TEST : 2D traj'); 

% Plot x -y plan : 
plot(data.odo_acc_x , data.odo_acc_y, '-'); hold on;
plot(data.kalman_acc_x , data.kalman_acc_y, '.');
title('x -y plan : kalman filter for accelerometer vs odometry');
legend('Odometry : Accelerometer', 'Kalman : Accelerometer');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data.kalman_acc_x;  data.odo_acc_x]),max([data.kalman_acc_x;  data.odo_acc_x])];
y_lim = [min([data.kalman_acc_y;  data.odo_acc_y]),max([data.kalman_acc_y;  data.odo_acc_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;