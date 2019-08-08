%% state estimation main


% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;

filename = '../logs/2019-07-01_14_34_31.csv';
M = csvread(filename, 1, 0);
col = size(M,2);

accel = M(:,2:4)./1024;  % INT32_ACCEL_FRAC
buf_a = accel;
gyro  = M(:,5:7)./4096;  % INT32_RATE_FRAC
angBody = M(:,8:10); 
rateBody = M(:,11:13);

optiPos = M(:,14:16);
cmd_thrust = M(:,17);

cmd_roll = M(:,18);
cmd_pitch = M(:,19);
cmd_yaw = M(:,20);
rpm(:,1:4) = M(:,21:24);
% rpm = rpm./max(max(rpm));
rpm = rpm * 2 * 3.142 /60; % return prop speed in rad/s
t = M(:,1)/512;
t = t - t(1,1);
dt = mean(gradient(t));
g =  9.81;

dr_state.x = M(:,25);
dr_state.y = M(:,26);

dr_cmd.roll  = M(:,27);
dr_cmd.pitch = M(:,28);


%% calc opti x, xd, xdd
st = 2;
[filt_a, optiAcc, optiVel]= optiData(optiPos, accel, t, dt, 150, st);

plotEverything(angBody, optiPos, optiVel, optiAcc, t);
%% drag co-efficients estimation (only lateral x and y)

rpmAvg = mean(rpm,2);
% rpmAvg = ones(size(rpm));
[kdx1, kdy1, kdx2, kdy2, kdx3, kdy3] = dragEst(angBody, filt_a, optiAcc, optiVel, rpmAvg, t);
 
