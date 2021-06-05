
%% Préférences figures
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesFontSize', 15);
set(groot, 'defaultLegendFontSize', 12);
set(groot, 'defaultLineLinewidth', 2);
set(groot, 'defaultLineMarkersize', 8);
format long;
% Part A : Plot accelerometers values 

[N_SIM, T_SIM, T, data] = read_log_metric();

f = figure('Name','accelerometer [m/s^2]');
plot(data.time(7:end), data.fit_cluster(7:end), '.'); hold on;
plot(data.time(7:end), movmean(data.fit_cluster(7:end),150), '-k'); hold on;
yl = yline(mean(data.fit_cluster(7:end-1)),'-.r')
yl.LineWidth=2;
grid on
xlabel('Time [s]')
ylabel('Flocking Metric [-]')
legend('Measurement', 'Moving mean', 'Mean =0.0612' )
title('Performance metric for the flocking algorithm ')

