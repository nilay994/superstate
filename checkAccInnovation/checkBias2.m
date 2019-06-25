%% RANSAC method for accelerometer bias and trend of accelerometer
% todo: yaw in the rotation matrix is it correct

% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;
filename = '../logs/2019-06-18_21_01_48.csv';  % plot(optiPos(:,1), optiPos(:,2)); axis equal, log of straight traj
% ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
M = csvread(filename, 1, 0);
col = size(M,2);

fid = fopen(filename);
a = textscan(fid,'%s',1);
C = strsplit(string(a),',');
fclose(fid);

accel = M(:,2:4)./1024;  % INT32_ACCEL_FRAC
buf_a = accel;
gyro  = M(:,5:7)./4096;  % INT32_RATE_FRAC
angBody = M(:,8:10); % find out if these are from optitrack or not
rateBody = M(:,11:13);

optiPos = M(:,14:16); % find out if using optiTrack height or not
cmd_thrust = M(:,17);
% todo: yaw in the rotation matrix is it correct

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



%% calc opti x, xd, xdd
st = 2;
[filt_a, optiAcc, optiVel]= optiData(optiPos, accel, t, dt, 150, st);

%%
thrust = zeros(length(t), 3);

for i = st:1:length(t)
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    
    thrust(i,:) = (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)])';
    
end

grndtruth = thrust;
[compensated_acc, parx, pary, parz] = compensator(grndtruth, accel, t);


%% 

acc_w = zeros(length(t), 3);
vel_w = zeros(length(t), 3);
pos_w = zeros(length(t), 3);

pos_w(1:st,:) = optiPos(1:st,:);
vel_w(1:st,:) = optiVel(1:st,:);
    
thrust = zeros(length(t), 3);

ab = zeros(length(t), 3);


for i = st:1:length(t)
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
    thrust(i,:) = (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)])';
    
    a_body(1) = compensated_acc(i,1);
    a_body(2) = compensated_acc(i,2);
    a_body(3) = compensated_acc(i,3);
    acc_t = [0;0;9.81] + R'* [a_body(1); a_body(2); a_body(3)];
    acc_w(i,1) = acc_t(1);
    acc_w(i,2) = acc_t(2);
    acc_w(i,3) = acc_t(3);
    
    vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w(i,1:3) .* dt);
    pos_w(i,1:3) = pos_w(i-1,1:3) + (vel_w(i,1:3) .* dt) + 0.5 .* acc_w(i,1:3) .* dt .* dt;
    
    
% %        METHOD1: Delftse  
%     az1 = thrust(i,3); %-9.81/(cos(theta) * cos(phi));
%     kdx1 = 0.5; kdy1 = 0.5;
%     acc_w(i,1) = (cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi))*az1 - kdx1 * vel_w(i-1,1);
%     acc_w(i,2) = (cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi))*az1 - kdy1 * vel_w(i-1,2);
%     acc_w(i,3) =  cos(theta) * cos(phi) * az1 + 9.81;
%     vel_w(i,1:3) = vel_w(i-1,1:3) + acc_w(i,1:3) .* dt;
%     pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt; 
% %     
%      ab(i,:) = R * (acc_w(i,:) - [0 0 9.81])';

end


%% now assume estimated body accelerations as ground truth and find innovation in acc

[compensated_acc2, parx1, pary1, parz1] = compensator(ab, accel, t);



%%

% top = figure; 
plot(pos_w(:,1), pos_w(:,2)); 
axis equal; hold on; grid on;
plot(optiPos(:,1), optiPos(:,2));
legend('filter1', 'gt'); 
xlabel('x (m)'); ylabel('y (m)')
text(optiPos(1,1), optiPos(1,2), 'start');
text(optiPos(end,1), optiPos(end,2), 'end');

%% 

figure; plot(t, optiPos(:,1)); hold on; plot(t, pos_w(:,1));
figure; plot(t, optiPos(:,2)); hold on; plot(t, pos_w(:,2));
figure; plot(t, optiPos(:,3)); hold on; plot(t, pos_w(:,3));