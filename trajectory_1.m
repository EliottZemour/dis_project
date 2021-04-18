function [x, y] = trajectory_1(x_0, y_0, theta_0)
% 
% Computes the ground true coordoniates of an e-puck during the first
% trajectory. 
%
% Arguments : 
%   - (x0, y0) is the coordinates of the e-puck at the begining of the
%   simulation
%
%   - theat0 is the orientation of the e-puck at the begining of the
%   simulation
%
% Return : 
%   the coordinates at each time step of the e-puck

% Simulation parameters
wheel_radius = 0.02; % m
inter_wheel_dist = 0.057; % m ; distance between the two wheels
time_step = 0.001; % 1ms
sim_duration = 115; % s

% Quantities
time = (0:time_step:sim_duration);
x = zeros(size(time));
y = zeros(size(time));
theta = zeros(size(time));

% Initial quantities
x(1) = x_0;
y(1) = y_0;
theta(1) = theta_0;

% Times at which occure a change in the trajectory
changes = [0, 3, 5, 14, 16, 50, 52, 70, 71.9, 72, 105, 106.9, 107, 115]; 

% angular velocities of the wheels on if portion of the trajectory
phi_dot_r = [6.28, 4, 6.28, 6.28, 6.28, 6.28, 6.28, 6.28, 0, 6.28, 6.28, 0, 6.28];
phi_dot_l = [6.28, 6.28, 6.28, 4, 6.28, 4, 6.28, 4, 0, 6.28, 4, 0, 6.28];

% Intergration of the trajectory

for k=1:length(changes)-1
 
    % Time references
    t_i = changes(i);
    t_f = changes(i+1);
    
    for t=t_i+time_step:t_f
        
        % Corresponding index
        index = int32(t/time_step) + 1;
        index_i = int32(t_i/time_step) + 1;

        % Initial coordinates at each changes
        x_i = x(index_i);
        y_i = y(index_i);
        theta_i = theta(index_i);

        % linear volcity of the e-puck
        v = (phi_dot_r(i) + phi_dot_l(i))/2 * wheel_radius;

        % angular velocity of the e-puck
        omega = (phi_dot_r(i) - phi_dot_l(i))/inter_wheel_dist * wheel_radius;

        % Coordinates
        if phi_dot_r(i) == phi_dot_l(i) 
            % Straight line
            theta(index) = theta_i;
            x(index) = cos(theta(index))*v*(t-t_i) + x_i;
            y(index) = sin(theta(index))*v*(t-t_i) + y_i;
        else 
            % Turn
            theta(index) = theta_i + omega*(t-t_i);
            x(index) = (sin(theta(index))-sin(theta_i))*v/omega + x_i;
            y(index) = (cos(theta_i)-cos(theta(index)))*v/omega + y_i;
        end
    end
end