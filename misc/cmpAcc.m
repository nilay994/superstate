%% TO centrifugal accelerations and coroilis forces
% todo: yaw in the rotation matrix is it correct

% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;
filename = '2019-04-30_18_02_45.csv';  % plot(optiPos(:,1), optiPos(:,2)); axis equal, log of straight traj
% ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
M = csvread(filename, 1, 0);
col = size(M,2);

fid = fopen(filename);
a = textscan(fid,'%s',1);
C = strsplit(string(a),',');
fclose(fid);

% M = M(3072:20482, :); % yaw seems the reason of drift. Also, Moment model
% in IMU filters seems to miss feedfoward terms, convering to gt takes
% time?

accel = M(:,2:4)./1024;
gyro  = M(:,5:7)./1024;
angBody = M(:,8:10); % find out if these are from optitrack or not
optiPos = M(:,11:13); % find out if using optiTrack height or not
cmd_thrust = M(:,14);
% todo: yaw in the rotation matrix is it correct

cmd_roll = M(:,15);
cmd_pitch = M(:,16);
cmd_yaw = M(:,17);
rpm(:,1:4) = M(:,18:21);
% rpm = rpm./max(max(rpm));
rpm = rpm * 2 * 3.142 /60; % return prop speed in rad/s
t = M(:,1)/512;
t = t - t(1,1);
dt = mean(gradient(t));
g =  9.81;

%% calc opti x, xd, xdd
st = 40;
[filt_a, optiAcc, optiVel]= optiData(optiPos, accel, t, dt, 150, st);
% verify if plots don't have out of bound values
plotEverything(angBody, optiPos, optiVel, optiAcc, t);


%% check eq1
phi_d = (phi - phi_prev)/dt;
theta_d = (theta - theta_prev)/dt;
psi_d = (psi - psi_prev)/dt;

phi_prev = phi;
theta_prev = theta;
psi_prev = psi;

R_omega = [1, 0, -sin(theta); 
           0, cos(phi), sin(phi) * cos(theta);
           0, -sin(phi), cos(phi) * cos(theta)];
  
       % peter corke model, try to see what was for your case
       % they used this model to convert world omega to body omegas
iW = [0        sin(psi)          cos(psi);             %inverted Wronskian
    0        cos(psi)*cos(the) -sin(psi)*cos(the);
    cos(the) sin(psi)*sin(the) cos(psi)*sin(the)] / cos(the);
% R_omega = eye(3);
omega = (R_omega) * [phi_d; theta_d; psi_d];

for i = 1:1:length(t)
    
    bodyAcc(i,:) = optiAcc(i,:) - cross(omega, optiVel(i,:));
    % cmp against accelerometer
    
end
%% check eq2
for i= 1:1:length(t)
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
    bodyAcc(i,:) = (R * optiAcc(i,:)')';
    % cmp against accelerometer
  
end