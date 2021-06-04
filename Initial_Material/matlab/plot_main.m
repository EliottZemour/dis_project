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



%% Part C : Plot the odometry computed using the accelerometer on Webots

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
%plot(data.time, data.pose_x); hold on;
plot(data.gps_x(3:end) , data.gps_y(3:end)); hold on;
plot(data.odo_acc_x, data.odo_acc_y);
title('trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('x [m]'); ylabel('y [m]');
y_lim = [min([data.odo_acc_y;  data.odo_acc_x]),max([data.odo_acc_y;  data.odo_acc_x])];




%% Part E : Plot the odometry computed using the wheel encoders on Webots

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using wheel encoders [Rad]'); 
subplot(3,1,[1 2]);hold on;

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

