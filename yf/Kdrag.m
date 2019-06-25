% Dealing the data from the simulator
close all;
clc;
clear;

GT = textread('gtOutput.txt'); % 120 Hz
%#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], 
% roll pitch yaw
sizeGT = size(GT);
lengthGT = sizeGT(1,1);
t = GT(:,1);
%%
optiPos = GT(:,2:4);

dt = mean(gradient(t));
g =  9.81;

optiVel(:,1) = smooth(gradient(optiPos(:,1))/dt, 150);
optiVel(:,2) = smooth(gradient(optiPos(:,2))/dt, 150);
optiVel(:,3) = smooth(gradient(optiPos(:,3))/dt, 150);

optiAcc(:,1) = smooth(gradient(optiVel(:,1))/dt, 150);
optiAcc(:,2) = smooth(gradient(optiVel(:,2))/dt, 150);
optiAcc(:,3) = smooth(gradient(optiVel(:,3))/dt, 150);


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
vel_body
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






%%




% Vicon = csvread('/home/yingfu/FMVIO/dataSet/ViconData.csv'); % 100Hz
% %#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []
% %attitude: body relative to world
% sizeVc = size(Vicon);
% lengthVc = sizeVc(1,1);
                     
IMU = textread('imuOutput.txt');  % 200Hz
%#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
sizeIMU = size(IMU);
lengthIMU = sizeIMU(1,1);

vel_body1 = zeros(lengthGT, 4);

% GT velocity in the world frame: GT(:, 9:11)
% GT data corresponding to Vicon data number: GT(:, 18), according to time stamps
for i = 2 : lengthGT
    
    %read the Quaternion
    Quaternion_BW = GT(i, 5:8);
    q0 = Quaternion_BW(1);
    q1 = Quaternion_BW(2);
    q2 = Quaternion_BW(3);
    q3 = Quaternion_BW(4);
              
    qx = [0 -q3 q2;
             q3 0 -q1;
            -q2 q1 0];
    % ?? R_BW transforms vectors from frame W to frame B
%     R_BW = q0^2*eye(3) + 2 * q0 * qx + qx * qx + [q1 q2 q3]' * [q1 q2 q3];
    q = quatnormalize([q0 q1 q2 q3]); %
    R_BW = quat2dcm(q);%
    
    velocity_world = (GT(i, 2:4) - GT(i - 1, 2:4)) / (GT(i, 1) - GT(i - 1, 1));
    velocity_body(i, 1) = GT(i, 1);
    velocity_body(i, 2:4) = (R_BW * velocity_world')';
    
    % TODO: How to do it without optiTrack
    % vel_body1(i, 1) = GT(i,1);
    % vel_body1(i, 2:4) = [(vel_body1(i-1,2) + IMU(i-1,5) * dt), (vel_body1(i-1,3) + IMU(i-1,6) * dt), (vel_body1(i-1,4) + IMU(i-1,7) * dt)];
    
    %[yaw,pitch,roll]=quat2angle([q0 q1 q2 q3], 'ZYX'); %rad
    %Euler(i, 1:4) = [roll,pitch,yaw, GT(i, 1)]; % time stamps
    
%     Euler(i, 1:3) * 180 / pi - GT(i, 9:11)
               
%     velocity_Body_GT(i, 1:3) = (R_WB' * GT(i, 9:11)')';
%     velocity_Body_GT(i, 4) = GT(i, 1); % time stamps
end


%% calculate drag parameter

% TODO: How do we calculate the k_drag when we do not know the bias of the acc of the Bebop?

accX = IMU(:, 5); % involve the bias from the GT
accXSmooth = smooth(accX);
accX2Smooth = smooth(accXSmooth);

[k_drag_X SX] = polyfit(velocity_body(:, 2), accX, 1) % only use the data in motion

fX = velocity_body(:, 2) * k_drag_X(1);
% T = table(velocity_Body_GT(:, 1), accX, fX, accX - fX, 'VariableNames',{'X','Y','Fit','FitError'});
errorXfitting = accX - fX;

accY = IMU(:, 6); % involve the bias, assume it is a constant TODO!!!

accYSmooth = smooth(accY);
accY2Smooth = smooth(accYSmooth);

[k_drag_Y SY] = polyfit(velocity_body(:, 3), accY, 1)

fY = velocity_body(:, 3) * k_drag_Y(1);
% T = table(velocity_Body_GT(:, 2), accY, fY, accY - fY, 'VariableNames',{'X','Y','Fit','FitError'});
errorYfitting = accY - fY;

% k_drag_X = -0.1745   -0.0921
% k_drag_Y = -0.2016   -0.0118

% %% save the .mat
% % Inertial sensor data:
% accMeas = accBody(200:16901, :);
% gyroMeas = gyroBody(200:16901, :);
% 
% % model parametererrorXfitting
% kDrag = [k_drag_X(1) k_drag_X(2);
%                k_drag_Y(1) k_drag_Y(2)];
% 
% % Ground Truth data:
% accBiasGT = accBiasBody;
% gyroBiasGT = gyroBiasBody;
% 
% EulerGT = Euler;  % attitude ("real body frame"(Vicon marker) to world frame)
% positionGT = [GT(:, 2:4) GT(:, 1)];
% velocityGT = [GT(:, 9:11) GT(:, 1)];
% 
% save imuMeas.mat accMeas gyroMeas kDrag
% save GT.mat accBiasGT gyroBiasGT EulerGT positionGT velocityGT velocity_Body_GT

%% figures
% truth = meas - bias

% figure(1)
% plot(velocity_Body_GT(:, 1));
% hold on;
% plot(velocity_Body_GT(:, 2));
% 
% figure(2)
% plot(accBody(:, 1), 'g');
% hold on;
% plot(accBody(:, 2), 'r');
% plot(accBody(:, 3), 'b');

% figure(3)
% plot(accBiasBody(:, 1), 'g'); 
% hold on;
% plot(accBiasBody(:, 2), 'r'); 
% plot(accBiasBody(:, 3), 'b');
% 
% figure(4)
% plot(accBody(200:16901, 1) - accBiasBody(:, 1), 'g'); % approximately  0.0806
% hold on;
% plot(accBody(200:16901, 2) - accBiasBody(:, 2), 'r'); % approximately  -0.1058
% plot(accBody(200:16901, 3) - accBiasBody(:, 3), 'b');

figure(5) 
subplot(1,2,1);
plot(accX,'g');
hold on;
plot(accX2Smooth','r');
plot(velocity_body(:, 2) * k_drag_X(1), 'b', 'linewidth', 4);
plot(-velocity_body(:, 2), 'm', 'linewidth', 1.0);
legend('acc meas X', 'accX Smooth', 'acc X=kDrag*velocity', 'velocity X Body GT');
xlabel('time');
title('X Body');
    
subplot(1,2,2);
plot(accY,'g');
hold on;
plot(accY2Smooth','r');
plot(velocity_body(:, 3) * k_drag_Y(1), 'b', 'linewidth', 4);
plot(-velocity_body(:, 3), 'm', 'linewidth', 1.0);
legend('acc meas Y', 'accY Smooth', 'acc Y=kDrag*velocity', 'velocity Y Body GT');
xlabel('time');
title('Y Body');

figure(7)
subplot(1,2,1);
plot(velocity_body(:, 2), accX,'.');
hold on;
plot([-6 6], [-6 6] * k_drag_X(1));
title('acc X = kDrag * velocityX');
xlabel('velocity X Body GT');
ylabel('acc meas X');
legend('data points', 'K Drag Velocity X Estimation');

subplot(1,2,2);
plot(velocity_body(:, 3), accY,'.');
hold on;
plot([-6 6], [-6 6] * k_drag_Y(1));
title('acc Y = kDrag * velocityY');
xlabel('velocity Y Body GT');
ylabel('acc meas Y');
legend('data points', 'K Drag Velocity Y Estimation');

% figure(6)
% subplot(1,2,1);
% plot(errorXfitting,'b');
% subplot(1,2,2);
% plot(errorYfitting,'b');


%% METHOD1: Think that accelerometer is good~~ (biasless)

% 1) convert body acc to world
% 2) integrate the world acc to find pos
accX = IMU(:, 5);
accY = IMU(:, 6); 
accZ = IMU(:, 7); 
dt = 1/960;
vel_w = zeros(lengthGT, 3);
pos_w = zeros(lengthGT, 3); 

pos_w(1,1:3) = GT(1, 2:4); 

for i = 2 : lengthGT
    
    %read the Quaternion
    Quaternion_BW = GT(i, 5:8);
    q0 = Quaternion_BW(1);
    q1 = Quaternion_BW(2);
    q2 = Quaternion_BW(3);
    q3 = Quaternion_BW(4);

    q = quatnormalize([q0 q1 q2 q3]); 
    R_BW = quat2dcm(q);
    
    acc_w = R_BW' * [accX(i,1); accY(i,1); accZ(i,1)];
    acc_w(3) = acc_w(3) - 9.81;
    
    vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w .* dt)';
    pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt; 
    
end

figure(9);


figure(10);
plot3(GT(:,2), GT(:,3), GT(:,4), '.b'); hold on;
plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3), '.r');
legend('GT', 'filter'); axis equal; grid on;
title('position');

%% METHOD2: Use drag

accX = IMU(:, 5);
accY = IMU(:, 6); 
accZ = IMU(:, 7); 
dt = mean(gradient(t));
vel_w = zeros(lengthGT, 3);
pos_w = zeros(lengthGT, 3); 
g = 9.81;
pos_w(1,1:3) = GT(1, 2:4); 
for i = 2 : lengthGT
    
    %read the Quaternion
    Quaternion_BW = GT(i, 5:8);
    q0 = Quaternion_BW(1);
    q1 = Quaternion_BW(2);
    q2 = Quaternion_BW(3);
    q3 = Quaternion_BW(4);

    q = quatnormalize([q0 q1 q2 q3]); 
    R_BW = quat2dcm(q);
    [yaw,pitch,roll] = quat2angle([q0 q1 q2 q3], 'ZYX'); %rad
    
    T(i) = g/(cos(pitch) * cos(roll));

    % TODO: trivial plot EVERYTHING
    acc_w = R_BW' * [0; 0; T(i)];
    % drag model
    acc_w(1) = acc_w(1) - 0.58 * vel_w(i-1,1);
    acc_w(2) = acc_w(2) - 0.58 * vel_w(i-1,2);
    acc_w(3) = acc_w(3) - 9.81;
    
    
    % az = accZ(i);
    % abx = az * sin(pitch);
    % aby = az * sin(-roll);
    % R_WB = R_BW';
    % ax = cos(yaw) * abx - sin(yaw) * aby - 0.58 * vel_w(i-1,1);
    % ay = sin(yaw) * abx + cos(yaw) * aby - 0.58 * vel_w(i-1,2);
    
    % does this mean that there is a need to model thrust here. 
    % acc_w = R_BW' * [accX(i,1); accY(i,1); accZ(i,1)];
    
    vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w .* dt)';
    pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt; 
    
end

figure(11);
plot3(GT(:,2), GT(:,3), GT(:,4), '.b'); hold on;
plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3), '.r');
legend('GT', 'filter'); axis equal; grid on;
title('position');

figure;
plot(t, T); hold on;

%% pull drag model via thrust model
close all;
IMU = textread('imuOutput.txt');  % 200Hz
GT = textread('gtOutput.txt');  % 200Hz
accX = IMU(:, 5);
accY = IMU(:, 6); 
accZ = IMU(:, 7); 
t = IMU(:,1);

optiPos = GT(:,2:4);

dt = mean(gradient(t));
lengthGT = size(GT, 1);

g =  9.81;

optiVel(:,1) = smooth(gradient(optiPos(:,1))/dt, 20);
optiVel(:,2) = smooth(gradient(optiPos(:,2))/dt, 20);
optiVel(:,3) = smooth(gradient(optiPos(:,3))/dt, 20);

optiAcc(:,1) = smooth(gradient(optiVel(:,1))/dt, 20);
optiAcc(:,2) = smooth(gradient(optiVel(:,2))/dt, 20);
optiAcc(:,3) = smooth(gradient(optiVel(:,3))/dt, 20);

vel_w = zeros(lengthGT, 3);
pos_w = optiPos(1,1:3);
acc_w = zeros(lengthGT, 3);

vel_w1 = zeros(lengthGT, 3);
pos_w1 = optiPos(1,1:3); 
acc_w1 = zeros(lengthGT, 3);

vel_w2 = zeros(lengthGT, 3);
pos_w2 = optiPos(1,1:3);
acc_w2 = zeros(lengthGT, 3);

vel_w3 = zeros(lengthGT, 3);
pos_w3 = optiPos(1,1:3);
acc_w3 = zeros(lengthGT, 3);

a_body = zeros(lengthGT, 3);
third = zeros(lengthGT, 3);

g = 9.81;
pos_w(1,1:3) = GT(1, 2:4); 
T = zeros(lengthGT, 1);

for i = 2 : lengthGT
    
    %read the Quaternion
    Quaternion_BW = GT(i, 5:8);
    q0 = Quaternion_BW(1);
    q1 = Quaternion_BW(2);
    q2 = Quaternion_BW(3);
    q3 = Quaternion_BW(4);

    q = quatnormalize([q0 q1 q2 q3]); 
    R_BW = quat2dcm(q);
    [yaw,pitch,roll] = quat2angle([q0 q1 q2 q3], 'ZYX'); %rad
    
    % filter 0
%     acc_w(i, 1:3) = (R_BW' * [IMU(i,5); IMU(i,6); IMU(i,7)])';
%     acc_w(i,3) = acc_w(i,3) - 9.81;  
%     vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w(i,1:3) .* dt);
%     pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt; 

    % filter 1 : Thrust = g/cos * cos
%     acc_w(i, 1:3) = (R_BW' * [0; 0; 9.81/(cos(pitch)*cos(roll))])';
%     acc_w(i,3) = acc_w(i,3) - 9.81;  
%     vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w(i,1:3) .* dt);
%     pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt; 

    % filter 1: drag model traditional - compensate after rotation
    thrust_temp = R_BW * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) + 9.81)];
    T(i) = thrust_temp(3);
    acc_t = (R_BW' * [0; 0; T(i)])';
    acc_w(i,1) = acc_t(1) - 0.58 * vel_w(i-1,1);
    acc_w(i,2) = acc_t(2) - 0.58 * vel_w(i-1,2);
    acc_w(i,3) = acc_t(3) - 9.81;
    vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w(i,1:3) .* dt);
    pos_w(i,1:3) = pos_w(i-1,1:3) + (vel_w(i,1:3) .* dt); % todo sync iter, second order taylor exp

    
    % filter 2 - BYU UPenn (best till now !~~?)
    % a_body is linearly scaled to v_body
    % yes, drone sees gravity in flight!! 
    gB = R_BW * [0;0;-9.81];
    kd = [-0.6855 0 0; 0 -0.5682 0; 0 0 0];
    a_body(i, 1:3) = (kd * R_BW * vel_w2(i-1, 1:3)' + gB)'; % plus some bias. 
    
    % FIND OUT, TODO: mostly it is the gravity but Kumar says its a constant bias
    % already compensated drag in body, dead reckon now
    acc_t = (R_BW' * [a_body(i,1); a_body(i,2); T(i)])';
    acc_w2(i,1) = acc_t(1);
    acc_w2(i,2) = acc_t(2);
    acc_w2(i,3) = acc_t(3) - 9.81;
    vel_w2(i,1:3) = vel_w2(i-1,1:3) + (acc_w2(i,1:3) .* dt);
    pos_w2(i,1:3) = pos_w2(i-1,1:3) + (vel_w2(i,1:3) .* dt); 
    
    % filter 3 - Real UPenn (without prop vel)
    kd = [-0.6855 0 0; 0 -0.5682 0; 0 0 0];
    third(i, 1:3) = (R_BW' * kd * R_BW * vel_w2(i-1, 1:3)')'; % plus some bias. 
    third(i, 3) = 0;
    % FIND OUT, TODO: mostly it is the gravity but Kumar says its a constant bias
    % already compensated drag in body, dead reckon now
    acc_t = R_BW' * [0;0;T(i)] - [0;0;9.81] + third(i,1:3)';
    acc_w3(i,1) = acc_t(1);
    acc_w3(i,2) = acc_t(2);
    acc_w3(i,3) = acc_t(3);
    vel_w3(i,1:3) = vel_w3(i-1,1:3) + (acc_w3(i,1:3) .* dt);
    pos_w3(i,1:3) = pos_w3(i-1,1:3) + (vel_w3(i,1:3) .* dt); 
    
%     % filter 3 - reverse reverse drag model - for thrust verify
%     % performs good in lateral acceleration but bad in thrust
%     % somehow gives less drift in z velocity 
%     acc_t = (R_BW' * [0; 0; T(i)])';
%     % compensate after rotation :( 
%     acc_w3(i,1) = acc_t(1) - 0.58 * vel_w3(i-1,1);
%     acc_w3(i,2) = acc_t(2) - 0.58 * vel_w3(i-1,2);
%     acc_w3(i,3) = acc_t(3) - 9.81;
%     vel_w3(i,1:3) = vel_w3(i-1,1:3) + (acc_w3(i,1:3) .* dt);
%     pos_w3(i,1:3) = pos_w3(i-1,1:3) + (vel_w3(i,1:3) .* dt); 
    
    % filter 4, velocity by analytical method, Kumar modelling
    
    
    % az = accZ(i);
    % abx = az * sin(pitch);
    % aby = az * sin(-roll);
    % R_WB = R_BW';
    % ax = cos(yaw) * abx - sin(yaw) * aby - 0.58 * vel_w(i-1,1);
    % ay = sin(yaw) * abx + cos(yaw) * aby - 0.58 * vel_w(i-1,2);
    
    % does this mean that there is a need to model thrust here. 
    % acc_w = R_BW' * [accX(i,1); accY(i,1); accZ(i,1)];
    

    
end
%%

% verify if rotated inertial is thrust : Yes it is, Unlike Bebop.
figure(100);
plot(t, IMU(:,7)); hold on;
plot(t, T)


% plot the dead reckoning by two methods 
figure(8); 

plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3)); 
axis equal; hold on; grid on;
plot3(pos_w2(:,1), pos_w2(:,2), pos_w2(:,3));
plot3(optiPos(:,1), optiPos(:,2), optiPos(:,3), '-b');
legend('filter1', 'filter2', 'gt'); 
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');


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


%% 
figure(10);
subplot(3,1,1);
plot(t, pos_w(:,1)); hold on; grid on;
plot(t, GT(:,2));

subplot(3,1,2);
plot(t, pos_w(:,2)); hold on; grid on;
plot(t, GT(:,3));

subplot(3,1,3);
plot(t, pos_w(:,3)); hold on; grid on;
plot(t, GT(:,4));



figure(11);
plot3(GT(:,2), GT(:,3), GT(:,4), '.b'); hold on;
plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3), '.r');
legend('GT', 'filter'); axis equal; grid on;
title('position');

figure; 
plot(t, IMU(:,7)); hold on;
plot(t, T);