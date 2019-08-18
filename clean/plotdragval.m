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
 

%% rnd loop - concludes that accelerometer's integrated velocity can't be used ever. 
% It is biased wrt to gt (sometimes) and high variance,
% if it was unbiased and low variance, it could still be used - since
% integrator removes this. 
r = zeros(length(t),3);
vel_w = zeros(length(t),3);
for i=2:1:length(t)
    r(i,1) = r(i-1,1) + filt_a(i,1) * 0.002;
    r(i,2) = r(i-1,2) + filt_a(i,2) * 0.002;
    r(i,3) = r(i-1,3) + filt_a(i,3) * 0.002;
end

% estimate accelerations and velocity in body frame  
for i = 2:1:length(t)
    phi = angBody(i,1);
    theta = angBody(i,2);
    psi = angBody(i,3);

    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    
    vel_wt = R' * r(i,:)';
    vel_w(i,:) = vel_wt';
end
