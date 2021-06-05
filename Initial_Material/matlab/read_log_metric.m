function [N_SIM, T_SIM, T, data] = read_log()
%% Read log file and create simulation variables
clc; clear all; close all;

temp = importdata('../controllers/flocking_super/flocking_metric.csv');

data = [];
ind = 0;
for f_ = str2mat(temp.colheaders)'
    ind = ind + 1;
    data.(strrep(f_',' ','')) = temp.data(:,ind);
end



N_SIM = length(data.time);
T_SIM = 1: N_SIM;
T = 0.016;

end