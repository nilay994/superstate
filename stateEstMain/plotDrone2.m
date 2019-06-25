%% TO find the psuedoaccelerations on body fixed frame
% todo: yaw in the rotation matrix is it correct

% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;
filename = '2019-02-19_13_57_58.csv'; % always worked only x drag
% filename = '2019-04-03_17_09_53.csv'; % high accelerations

% filename = '2019-04-04_19_13_09.csv';
% filename = '2019-04-05_18_22_24.csv'; % these are too long and don't give drag coeffs.
% filename = '2019-04-06_13_52_39.csv'; % accel_scaled = 9.8 but unscaled/512 gives different values
% ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
M = csvread(filename, 1, 0);
col = size(M,2);

fid = fopen(filename);
a = textscan(fid,'%s',1);
C = strsplit(string(a),',');
fclose(fid);

accel = M(:,2:4)./512;
gyro  = M(:,5:7)./512;
angBody = M(:,11:13); % find out if these are from optitrack or not
optiPos = M(:,14:16);
t = M(:,1)/512;


for i=1:1:col
     % figure('Name', C(i));
     % plot(M(:,i));
end

%% plot opti x, xd, xdd
% STEP2: PLOT PARSED DATA

dt = mean(gradient(t));
g =  9.81;

% filtering a lot loses data, integrating it back introduces drift
optiVel(:,1) = smooth(gradient(optiPos(:,1))/dt, 150);
optiVel(:,2) = smooth(gradient(optiPos(:,2))/dt, 150);
optiVel(:,3) = smooth(gradient(optiPos(:,3))/dt, 150);
close 
optiAcc(:,1) = smooth(gradient(optiVel(:,1))/dt, 150);
optiAcc(:,2) = smooth(gradient(optiVel(:,2))/dt, 150);
optiAcc(:,3) = smooth(gradient(optiVel(:,3))/dt, 150);

% filter the body accelerations
filter = mkfilter(2, 5, 'bessel'); % 2Hz cutoff, 5th order bessel filter
filt_a(:,1) = lsim(filter, accel(:,1), t);
filt_a(:,2) = lsim(filter, accel(:,2), t);
filt_a(:,3) = lsim(filter, accel(:,3), t);



%% 
figure;
subplot(5,3,1);
plot(t, angBody(:,2));
title('\theta');
xlabel('secs');

subplot(5,3,2);
plot(t, angBody(:,1));
title('\phi');
xlabel('secs');

subplot(5,3,3);
plot(t, angBody(:,3));
title('\psi');
xlabel('secs');

subplot(5,3,4);
plot(t, optiPos(:,1));
title('optiPos_x');
xlabel('secs');

subplot(5,3,5);
plot(t, optiPos(:,2));
title('optiPos_y');

subplot(5,3,6);
plot(t, optiPos(:,3));
title('optiPos_z');

subplot(5,3,7);
plot(t, optiVel(:,1));
title('optiVel_x');

subplot(5,3,8);
plot(t, optiVel(:,2));
title('optiVel_y');

subplot(5,3,9);
plot(t, optiVel(:,3));
title('optiVel_z');

subplot(5,3,10);
plot(t, optiAcc(:,1));
title('optiAcc_x');

subplot(5,3,11);
plot(t, optiAcc(:,2));
title('optiAcc_y');

subplot(5,3,12);
plot(t, optiAcc(:,3));
title('optiAcc_z');

% instead of using body acc to compare against world acc, 
% use body angles to predict body acc and then convert body to world acc

% thrust causes an upward acceleration (but is a downward force?) 
% since upward, give "T = -g:"
% assuming accelerations in lateral directions on body frame (drag etc) are
% zero, only map thrust to world accelerations
T = - g;
wRb_ax = T.* (cos(angBody(:,1)).*sin(angBody(:,2)).*cos(angBody(:,3)) + sin(angBody(:,1)).*sin(angBody(:,3)));
subplot(5,3,13);
plot(t, wRb_ax);
title('droneAcc_x in world frame');

subplot(5,3,14);
wRb_ay = T.* (cos(angBody(:,1)).*sin(angBody(:,2)).*sin(angBody(:,3)) - sin(angBody(:,1)).*cos(angBody(:,3)));
plot(t, wRb_ay);
title('droneAcc_y in world frame');

subplot(5,3,15);
wRb_az = T .* cos(angBody(:,1)).* cos(angBody(:,2));
plot(t, wRb_az);
title('droneAcc_z in world frame');

%% ASSUMING hover, estimate the accelerations in world frame

figure(3);

subplot(3,1,1);
plot(t, optiAcc(:,1)); hold on; grid on;
plot(t, wRb_ax); xlabel('time'); ylabel('acc (m/s^2)');
legend('acc_x^E, optiTrack', 'R_B^E acc_x^B, ahrs imu');

subplot(3,1,2);
plot(t, optiAcc(:,2)); hold on; grid on;
plot(t, wRb_ay); xlabel('time'); ylabel('acc (m/s^2)');
legend('acc_y^E, optiTrack', 'R_B^E acc_y^B, ahrs imu');

subplot(3,1,3);
plot(t, optiAcc(:,3)); hold on; grid on;
plot(t, wRb_az - 9.81); xlabel('time'); ylabel('acc (m/s^2)');
legend('acc_z^E, optiTrack', 'R_B^E acc_z^B, ahrs imu');
sgtitle('inertial accelerations from optiTrack v/s accelerometer assuming hover');


%%  now calc body accelerations and velocity from optitrack and imu accelerations

% STEP1: subtract gravity world frame, to get forces in z direction on body
% frame.
% optiAcc(:,3) = optiAcc(:,3) - g; %TODO: currently looks right

% STEP2: initialize rotation matrix to convert from World to Body
bodyAcc_ot = zeros(length(t), 3);
bodyAcc_imu = zeros(length(t), 3);
bodyVel_ot = zeros(length(t), 3);
bodyVel_imu = zeros(length(t), 3);

% STEP3: estimate accelerations and velocity in body frame  
for i = 2:1:length(t)
    
    phi = angBody(i,1);
    theta = angBody(i,2);
    psi = angBody(i,3);
      
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    
    bodyAcc_t = R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)]; % add grav here NED
    bodyVel_t = R * [optiVel(i,1); optiVel(i,2); optiVel(i,3)];
    
    bodyAcc_ot(i,1) = bodyAcc_t(1);
    bodyVel_ot(i,1) = bodyVel_t(1);
    
    bodyAcc_ot(i,2) = bodyAcc_t(2);
    bodyVel_ot(i,2) = bodyVel_t(2);
    
    bodyAcc_ot(i,3) = bodyAcc_t(3);
    bodyVel_ot(i,3) = bodyVel_t(3);
    
    
    % METHOD2: Use accelerations from IMU - complete bs: bodyVel_imu -
    % compensate for drag right from here. There is no way to estimate drag
    % co-efficients in flight. Do it offboard, optiTrack mandatory. 
    % ref: BYU and UPenn.
    bodyAcc_imu(i,1) = accel(i,1);
    bodyVel_imu(i,1) = bodyVel_imu(i-1,1) + bodyAcc_imu(i,1) * dt;
    
    bodyAcc_imu(i,2) = accel(i,2);
    bodyVel_imu(i,2) = bodyVel_imu(i-1,2) + bodyAcc_imu(i,2) * dt;
    
    bodyAcc_imu(i,3) = accel(i,3); % no need of augmenting gravity here
    bodyVel_imu(i,3) = bodyVel_imu(i-1,3) + bodyAcc_imu(i,3) * dt;
    
end

% STEP4: Plot the accelerations and velocities for imu and ot and compare
% them
figure(5);
subplot(3,2,1);
plot(t, bodyAcc_ot(:,1)); hold on;
plot(t, bodyAcc_imu(:,1)); 
legend('optiTrack', 'accelerometer');
title('bodyAcc_x'); grid on;
xlabel('time (s)'); ylabel('a^B_x (m/s^2)');

subplot(3,2,2);
plot(t, bodyVel_ot(:,1)); hold on;
plot(t, bodyVel_imu(:,1));
legend('optiTrack', 'accelerometer');
title('bodyVel_x'); grid on;
xlabel('time (s)'); ylabel('v^B_x (m/s)');

subplot(3,2,3);
plot(t, bodyAcc_ot(:, 2));  hold on;
plot(t, bodyAcc_imu(:,2)); 
legend('optiTrack', 'accelerometer');
title('bodyAcc_y'); grid on;
xlabel('time (s)'); ylabel('a^B_y (m/s^2)');

subplot(3,2,4);
plot(t, bodyVel_ot(:,2)); hold on;
plot(t, bodyVel_imu(:,2));
title('bodyVel_y'); grid on;
legend('optiTrack', 'accelerometer');
xlabel('time (s)'); ylabel('v^B_y (m/s)');

subplot(3,2,5); 
plot(t, bodyAcc_ot(:,3));  hold on;
plot(t, bodyAcc_imu(:,3));
legend('optiTrack', 'accelerometer');
title('bodyAcc_z'); grid on;
xlabel('time (s)'); ylabel('a^B_z (m/s^2)');

subplot(3,2,6);
plot(t, bodyVel_ot(:,3)); hold on;
plot(t, bodyVel_imu(:,3));
title('bodyVel_z'); grid on;
legend('optiTrack', 'accelerometer');
xlabel('time (s)'); ylabel('v^B_z (m/s)');

% compare estimation of drag co-efficient using ot and using imu
% drag co-efficient in x direction
figure(6);
subplot(1,2,1);
plot(bodyVel_ot(:,1), bodyAcc_ot(:,1), '.'); 
hold on; grid on;

p_x = polyfit(bodyAcc_ot(:,1), bodyVel_ot(:,1), 1);
yaxis = polyval(p_x, bodyVel_ot(:,1));
plot(bodyVel_ot(:,1), yaxis);
xlabel('vel^B_x'); ylabel('acc^B_x');
title('OptiTrack vel vs acc in body frame');

kdx = -p_x(1);

subplot(1,2,2);
plot(bodyVel_imu(:,1), bodyAcc_imu(:,1), '.'); 
hold on; grid on;

p_x = polyfit(bodyAcc_imu(:,1), bodyVel_imu(:,1), 1);
yaxis = polyval(p_x, bodyVel_imu(:,1));
plot(bodyVel_imu(:,1), yaxis);
xlabel('vel^B_x'); ylabel('acc^B_x');
title('IMU vel_x vs acc in body frame');

% drag co-efficient in y direction
figure(7);
subplot(1,2,1);
plot(bodyVel_ot(:,2), bodyAcc_ot(:,2), '.'); 
hold on; grid on;

p_y = polyfit(bodyAcc_ot(:,2), bodyVel_ot(:,2), 1);
yaxis = polyval(p_y, bodyVel_ot(:,2));
plot(bodyVel_ot(:,2), yaxis);
xlabel('vel^B_y'); ylabel('acc^B_y');
title('OptiTrack vel_y vs acc in body frame');

kdy = -p_y(1);

subplot(1,2,2);
plot(bodyVel_imu(:,2), bodyAcc_imu(:,2), '.'); 
hold on; grid on;

p_y = polyfit(bodyAcc_imu(:,2), bodyVel_imu(:,2), 1);
yaxis = polyval(p_y, bodyVel_imu(:,2));
plot(bodyVel_imu(:,2), yaxis);
xlabel('vel^B_y'); ylabel('acc^B_y');
title('IMU vel vs acc in body frame');


%% 
acc_w = zeros(length(t), 3);
vel_w = zeros(length(t), 3);
pos_w = zeros(length(t), 3);

acc_w2 = zeros(length(t), 3);
vel_w2 = zeros(length(t), 3);
pos_w2 = zeros(length(t), 3);

acc_w3 = zeros(length(t), 3);
vel_w3 = zeros(length(t), 3);
pos_w3 = zeros(length(t), 3);

acczBody = zeros(length(t), 3);

pos_w(1,:) = optiPos(1,:);
pos_w(2,:) = optiPos(2,:);

for i = 2:1:length(t)
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
    % METHOD1: Convert filtered accelerometer to world frame and dead reckon
    % suggestion: maybe yaw drift is propagating reference: 
    % Kumar: Inertial Yaw-Independent Velocity and Attitude Estimation for High-Speed Quadrotor Flight
    
    acc = R' * accel(i,:)';
    % remove pseudo gravity
    acc(3) = acc(3) + g;
    acc_w(i,1:3) = acc';
    vel_w(i,1:3) = vel_w(i-1,1:3) + (acc .* dt)';
    pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt;   
    
    
    % METHOD2: Make use of drag co-efficients
%     az2 =  9.81 / (cos(theta) * cos(phi));
%     abx2 =  sin(-theta) * az2;
%     aby2 =  sin(phi)    * az2;
%     
%     ax2 =  cos(psi) * abx2 - sin(psi) * aby2 - vel_w2(i-1,1) * kdx;
%     ay2 =  sin(psi) * abx2 + cos(psi) * aby2 - vel_w2(i-1,2) * kdy;
%     acc_w2 = [ax2; ay2; az2];
    acc = R' * [0; 0; -g/(cos(theta) * cos(phi))];
    acc(3) = acc(3) + g;
    % drag model compensating in world frame
    acc_w2(i,1) = acc(1) - vel_w2(i-1,1) * kdx;
    acc_w2(i,2) = acc(2) - vel_w2(i-1,2) * kdy; %TODO: change back to kdy once better logs
    acc_w2(i,3) = acc(3);
    
    vel_w2(i,1:3) = vel_w2(i-1,1:3) + (acc .* dt)';
    pos_w2(i,1:3) = pos_w2(i-1,1:3) + vel_w2(i,1:3) .* dt;
    
    
    % filter 3, assume thrust is known, use thrust from optiTrack augment with gravity
    acczBody(i,1:3) = R * [0; 0; (optiAcc(i,3)-9.81)]; %  
    acc = R' * [0; 0; acczBody(i,3)];
    acc(3) = acc(3) + g;
    % drag model
    acc_w3(i,1) = acc(1) - vel_w3(i-1,1) * kdx;
    acc_w3(i,2) = acc(2) - vel_w3(i-1,2) * kdy; %TODO: change back to kdy once better logs
    acc_w3(i,3) = acc(3);
    
    vel_w3(i,1:3) = vel_w3(i-1,1:3) + (acc .* dt)';
    pos_w3(i,1:3) = pos_w3(i-1,1:3) + vel_w3(i,1:3) .* dt;
    
    % filter 4, assume thrust is know, also augment drag forces now~~
    % refer kdrag.m
end

% plot the dead reckoning by two methods 
figure(8); 

plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3)); 
axis equal; hold on; grid on;
plot3(pos_w2(:,1), pos_w2(:,2), pos_w2(:,3));
plot3(optiPos(:,1), optiPos(:,2), optiPos(:,3), '-b');
legend('filter1', 'filter2', 'gt'); 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z(m)');


figure(9);
sgtitle('position');
subplot(3,1,1);
plot(t, pos_w(:,1)); 
hold on; grid on; 
plot(t, pos_w2(:,1));
plot(t, pos_w3(:,1));
plot(t, optiPos(:,1));
legend('filter1', 'filter2','filter3','gt'); xlabel('time (s)'); ylabel('x (m)');

subplot(3,1,2);
plot(t, pos_w(:,2)); 
hold on; grid on; 
plot(t, pos_w2(:,2));
plot(t, pos_w3(:,2));
plot(t, optiPos(:,2));
legend('filter1', 'filter2', 'filter3','gt'); xlabel('time (s)'); ylabel('y (m)');

subplot(3,1,3);
plot(t, pos_w(:,3)); 
hold on; grid on; 
plot(t, pos_w2(:,3));
plot(t, pos_w3(:,3));
plot(t, optiPos(:,3));
legend('filter1', 'filter2', 'filter3','gt'); xlabel('time (s)'); ylabel('z (m)');


% check if velocity is converging using the new model. It did not converge when 
% 1) integrated filtered accelerometer, lost too much information, zero
% velocity (above were in body frame)
% 2) This method finally gave something since it used the dynamic model.
% (in inertial frame). Papers I have read have also only cared out this
% inertial frame while body frame velocities (which are used in calculating drag)
% aren't used at all.

figure(10);
sgtitle('lateral velocity');
subplot(3,1,1);
plot(t, vel_w(:,1)); 
hold on; grid on; 
plot(t, vel_w2(:,1));
plot(t, vel_w3(:,1));
plot(t, optiVel(:,1));
legend('filter1', 'filter2', 'filter3', 'gt'); xlabel('time (s)'); ylabel('v_x (m/s)');

subplot(3,1,2);
plot(t, vel_w(:,2)); 
hold on; grid on; 
plot(t, vel_w2(:,2));
plot(t, vel_w3(:,2));
plot(t, optiVel(:,2));
legend('filter1', 'filter2', 'filter3', 'gt'); xlabel('time (s)'); ylabel('v_y (m/s)');

subplot(3,1,3);
plot(t, vel_w(:,3)); 
hold on; grid on; 
plot(t, vel_w2(:,3));
plot(t, vel_w3(:,3));
plot(t, optiVel(:,3));
legend('filter1', 'filter2', 'filter3', 'gt'); xlabel('time (s)'); ylabel('v_z (m/s)');

figure(11);
sgtitle('acceleration');
subplot(3,1,1);
plot(t, acc_w(:,1)); 
hold on; grid on; 
plot(t, acc_w2(:,1));
plot(t, acc_w3(:,1));
plot(t, optiAcc(:,1));
legend('filter1', 'filter2', 'filter3', 'gt'); xlabel('time (s)'); ylabel('a_x (m/s^2)');

subplot(3,1,2);
plot(t, acc_w(:,2)); 
hold on; grid on; 
plot(t, acc_w2(:,2));
plot(t, acc_w3(:,2));
plot(t, optiAcc(:,2));
legend('filter1', 'filter2', 'filter3', 'gt'); xlabel('time (s)'); ylabel('a_y (m/s^2)');

subplot(3,1,3);
plot(t, acc_w(:,3)); 
hold on; grid on; 
plot(t, acc_w2(:,3));
plot(t, acc_w3(:,3));
plot(t, optiAcc(:,3));
legend('filter1', 'filter2', 'filter3', 'gt'); xlabel('time (s)'); ylabel('a_z (m/s^2)');


%% METHOD 3: also use gyroscopic terms, moments EKF time





%%

% doubts, 
% why is it in body frame - YingFu resolved the components for you
% can it be also in world frame psuedo force (inertial frame)
% try out propagation and compare pred with GT (problem in Z position)
% Why isn't body thrust of g/(cos*cos) used in drag estimation?
% is the sensor and body CoM on same axes
% apply 5th order Bessel filter, 2Hz cutoff
% obviously I can't dead reckon the accelerations: body to world
% accelerations, double integration, wouldn't work. Vibration sensor is
% noisy. 
% We have three types of body accelerations. 
% 1) rotated double derivative optitrack.
% 2) accelerometer body measurements
% 3) observer design: predict accelerations using AHRS and thrust profile.

% Why are we using the velocity in the world frame and multiplying it with
% the drag? read a few papers? :( - SiHao Sun, YingFu
% todo: Faessler uzh-rpg/drone-controller document for math and rigid body dynamics 
% todo: use Kdrag.m equations here using dead reckoning of accelerometer
% readings (DONE)
% todo: try Shuo's equations in Kdrag.m (DONE)
% todo: try Shou's equations here (DONE)
% todo: shuo says control in drag is already included. (thrust component isn't -
% Yingfu)
% todo: compare zurich ROS and UPenn kumar to understand their strategies
% todo: WRITE!! on sticky with printouts of important papers
% todo: take meeting next week
% doubt: No thrust (body z) compensation for drag? 
% is the hypothesis correct? There is a drift in the AHRS and hence there
% is drift in the filter?  No. There are drag forces and external
% disturbances. If the EoM are integrated using Euler (Kumar) do we expect
% a better result due to "virtual coupling" or do we call "dual quaternions" for
% rescue
% todo: Check gyroscopic effects
% minimum pitch angle so that the wake of one doesn't affect the other,
% maybe better in trashcan than bebop
% check how inertial conservative forces (body resolve drag) works
% todo: check drag coefficient only from IMU - (FAILS)
% todo: need to check low pass filter method (which could also mean an integrator).
% It will surely give non zero estimated velocities. It is not a problem
% for the time being

% interesting expt: Pitching produces acceleration in x axis. However it
% seems that the drone doesn't accelerate, but keeps going in constant
% velocity while pitching. This would imply that there are some forces that
% are making the net lateral acceleration = 0. Hence we equate the drag
% force against thrust force. However to try:
% 1) paparazzi: constant pitch
% 2) see if acceleration in x axis via optiTrack

% find out how much yaw drift causing drift in actual model
% find out if 9.81 accelerometer when stationary to know more about biases:
% Yes, it throws -8.8 wtf!, need to do something about it. AHRS calib wiki 




% todo: identify thrust model, 
% T = kT * (w.^2) + kVz * V(:,3) * w.^2 + kV * (sqrt(V(:,1).^2 + V(:,2)^2)^2)
% a least square fit is possible, no surprises here, maybe scale the rpm

% todo: perform various maneuvers, take new plot, maybe learn more about
% waypoints




%% Peter Corke book
ax=10;
ay=20;
az=30;

Cx = [1 0 0; 0 cosd(ax) -sind(ax); 0 sind(ax) cosd(ax)];
Cy = [cosd(ay) 0 sind(ay); 0 1 0; -sind(ay) 0 cosd(ay)];
Cz = [cosd(az) -sind(az) 0; sind(az) cosd(az) 0; 0 0 1];


%% earth to body shuo

    R_E_B = [cos(COMP.THETA(i))*cos(COMP.PSI(i)) cos(COMP.THETA(i))*sin(COMP.PSI(i)) -sin(COMP.THETA(i));...
        sin(COMP.PHI(i))*sin(COMP.THETA(i))*cos(COMP.PSI(i))-cos(COMP.PHI(i))*sin(COMP.PSI(i))...
        sin(COMP.PHI(i))*sin(COMP.THETA(i))*sin(COMP.PSI(i))+cos(COMP.PHI(i))*cos(COMP.PSI(i)) sin(COMP.PHI(i))*cos(COMP.THETA(i));...
        cos(COMP.PHI(i))*sin(COMP.THETA(i))*cos(COMP.PSI(i))+sin(COMP.PHI(i))*sin(COMP.PSI(i))...
        cos(COMP.PHI(i))*sin(COMP.THETA(i))*sin(COMP.PSI(i))-sin(COMP.PHI(i))*cos(COMP.PSI(i)) cos(COMP.PHI(i))*cos(COMP.THETA(i))];

%% integrate optiTrack back
op_acc = zeros(length(t), 3);
op_vel = zeros(length(t), 3);
op_pos = zeros(length(t), 3);

op_vel(1,1:3) = optiVel(1,1:3);
op_pos(1,1:3) = optiPos(1,1:3);

for i=2:1:length(t)
    op_acc(i-1,1:3) = optiAcc(i-1,1:3);
    op_vel(i,1:3) = op_vel(i-1,1:3) + op_acc(i-1,1:3)*dt;
    op_pos(i,1:3) = op_pos(i-1,1:3) + op_vel(i-1,1:3)*dt + 0.5*op_acc(i-1,1:3)*(dt^2);
end
figure;
plot3(op_pos(:,1), op_pos(:,2), op_pos(:,3), '.b'); 
hold on; grid on; axis equal;
plot3(optiPos(:,1), optiPos(:,2), optiPos(:,3), '.r');

figure;
subplot(2,1,1);
plot(t, op_vel(:,1)); hold on; grid on;
plot(t, optiVel(:,1));
subplot(2,1,2);
plot(t, op_vel(:,2)); hold on; grid on;
plot(t, optiVel(:,2));

