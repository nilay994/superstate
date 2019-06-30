%% to find pitch angles setpoint feedfwd?

% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;

%
filename = '/home/nilay/Downloads/2019-06-27_17_22_54.csv';  % plot(optiPos(:,1), optiPos(:,2)); axis equal, log of straight traj
% ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
M = csvread(filename, 1, 0);
col = size(M,2);


accel = M(:,2:4)./1024;  % INT32_ACCEL_FRAC
buf_a = accel;
gyro  = M(:,5:7)./4096;  % INT32_RATE_FRAC
angBody = M(:,8:10); % phi, theta, psi: roll, pitch, yaw
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


%% calc optiTrack data: 
% NOTE: using smooth creates jumps at the head and the tail of the vector 
% if a cropped flight data is used. 
% (cropped flight data is essential for thrust modelling, no one is starting and stopping logs for me in hover position)
windowSize = 40; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

optiVel(:,1) = filter(b,a, gradient(optiPos(:,1))/dt);
optiVel(:,2) = filter(b,a, gradient(optiPos(:,2))/dt);
optiVel(:,3) = filter(b,a, gradient(optiPos(:,3))/dt);

optiAcc(:,1) = filter(b,a, gradient(optiVel(:,1))/dt);
optiAcc(:,2) = filter(b,a, gradient(optiVel(:,2))/dt);
optiAcc(:,3) = filter(b,a, gradient(optiVel(:,3))/dt);


% filter the body accelerations  
% 2Hz cutoff, 5th order bessel filter
filter_acc = mkfilter(2, 5, 'bessel');
filt_a(:,1) = lsim(filter_acc, accel(:,1), t);
filt_a(:,2) = lsim(filter_acc, accel(:,2), t);
filt_a(:,3) = lsim(filter_acc, accel(:,3), t);

st = 2;
[filt_a, optiAcc, optiVel]= optiData(optiPos, accel, t, dt, 150, st);
%% check if thrust matches (in case of incorrect altitude pprz) 
T = thrustMatch(angBody, optiAcc, filt_a, t);

acc_w = zeros(length(t), 3);
vel_w = zeros(length(t), 3);
pos_w = zeros(length(t), 3);

st = 40;
pos_w(1:st,:) = optiPos(1:st,:);
vel_w(1:st,:) = optiVel(1:st,:);

velBody = zeros(length(t), 3);
newT = zeros(length(t), 1);
% check what happens with velbody
for i = (st-2):1:length(t)
    
    phi = angBody(i,1);
    theta = angBody(i,2);
    psi = angBody(i,3);
      
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    
    velBody(i,1:3) = (R * [optiVel(i,1); optiVel(i,2); optiVel(i,3)])'; % laterals shouldn't be zero
    % newT(i,1) = T(i) +  * rssq([velBody(i,1),velBody(i,2)])^2; % time to match with body z (accelerometer)
    a_body = (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)]);
    acc_t = ([0;0;9.81] + R' * [a_body(1); a_body(2); a_body(3)])';
    acc_w(i,1) = acc_t(1);
    acc_w(i,2) = acc_t(2);
    acc_w(i,3) = acc_t(3);
    
    vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w(i,1:3) .* dt);
    pos_w(i,1:3) = pos_w(i-1,1:3) + (vel_w(i,1:3) .* dt);
    
end



%% TODO: check sanity of optiTrack integrate back, which will tell you what your thrust model should be giving out. 
% what you see currently might not be the thrust but something else - which



% is scary. it might be some drag thing or some velBody thing