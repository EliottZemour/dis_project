function [data] = read_log_metric_formation()
%% Read log file and create simulation variables
clc; clear all; close all;

temp = importdata('../controllers/formation_super/formation_metric.csv');

data = [];
ind = 0;
for f_ = str2mat(temp.colheaders)'
    ind = ind + 1;
    data.(strrep(f_',' ','')) = temp.data(:,ind);
end


end