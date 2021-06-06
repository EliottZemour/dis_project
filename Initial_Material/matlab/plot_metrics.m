
%% Préférences figures
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesFontSize', 11);
set(groot, 'defaultLegendFontSize', 12);
set(groot, 'defaultLineLinewidth', 2);
set(groot, 'defaultLineMarkersize', 8);
format long;
% Part A : Plot accelerometers values 

[data1] = read_log_metric_localization();
[data2] = read_log_metric_flocking();
[data3] = read_log_metric_formation();

f = figure('Name','accelerometer [m/s^2]');
semilogy(data1.time(100:end), data1.metric_enc(100:end), '.'); hold on;
semilogy(data1.time(100:end), data1.metric_acc(100:end), '.'); hold on;
semilogy(data1.time(100:end), data1.metric_kalman_acc(100:end), '.'); hold on;
%semilogy(data2.time(10:end), data2.fit_cluster(10:end), '.'); hold on;
%plot(data.time(100:end), movmean(data.fit_formation(7:end),150), '-k'); hold on;
yl1 = yline(mean(data1.metric_enc(100:end-1)),'-.')
yl2 = yline(mean(data1.metric_acc(100:end-1)),'-.')
yl3 = yline(mean(data1.metric_kalman_acc(100:end-1)),'-.')
yl1.LineWidth=2;
yl2.LineWidth=2;
yl3.LineWidth=2;
grid on
xlabel('Time [s]')
ylabel('Localization Metric [m]')
legend('Wheel encoder', 'Accelerometer', 'Kalman Acc' )
title('Performance metric for the localization odometry ')

figure;
plot(data2.time(10:end), data2.fit_cluster(10:end), '.')
