%% TO find the psuedoaccelerations on body fixed frame
% todo: yaw in the rotation matrix is it correct

% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;
filename = '../logs/log_file_00013.csv';  % plot(optiPos(:,1), optiPos(:,2)); axis equal, log of straight traj
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

accel = M(:,2:4)./1024;  % INT32_ACCEL_FRAC
buf_a = accel;
gyro  = M(:,5:7)./4096;  % INT32_RATE_FRAC
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
st = 400;
[filt_a, optiAcc, optiVel]= optiData(optiPos, accel, t, dt, 150, st);
% verify if plots don't have out of bound values
plotEverything(angBody, optiPos, optiVel, optiAcc, t);


%% check if thrust matches (in case of incorrect altitude pprz) 
T = thrustMatch(angBody, optiAcc, filt_a, t);

%% drag co-efficients estimation (only lateral x and y)

rpmAvg = mean(rpm,2);
% rpmAvg = ones(size(rpm));
[kdx1, kdy1, kdx2, kdy2, kdx3, kdy3] = dragEst(angBody, filt_a, optiAcc, optiVel, rpmAvg, t);

%% or read from old dataset to prevent overfitting
rpmAvg = mean(rpm,2);
[kdx1, kdy1, kdx2, kdy2, kdx3, kdy3] = readDragCo();

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

pos_w(st-1,:) = optiPos(st-1,:);
pos_w(st,:) = optiPos(st,:);

vel_w(st-1,:) = optiVel(st-1,:);
vel_w(st,:) = optiVel(st,:);

pos_w2(st-1,:) = optiPos(st-1,:);
pos_w2(st,:) = optiPos(st,:);

vel_w2(st-1,:) = optiVel(st-1,:);
vel_w2(st,:) = optiVel(st,:);

pos_w3(st-1,:) = optiPos(st-1,:);
pos_w3(st,:) = optiPos(st,:);

vel_w3(st-1,:) = optiVel(st-1,:);
vel_w3(st,:) = optiVel(st,:);

phi_prev   = angBody(1,1);
theta_prev = angBody(1,2);
psi_prev   = angBody(1,3);
    
thrust = zeros(length(t), 3);
Tnew = zeros(length(t), 1);
diffA = zeros(length(t), 3);


for i = st:1:length(t)
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
      % emulated PnP
    % update the positions and velocities atleast 20 times
    if rem(i,round(length(t)/20)) == 0 
        pos_w(i-1,:)  = optiPos(i-1,:);
        pos_w2(i-1,:) = optiPos(i-1,:);
        pos_w3(i-1,:) = optiPos(i-1,:);
        
        vel_w(i-1,:)  = optiVel(i-1,:);
        vel_w2(i-1,:) = optiVel(i-1,:);
        vel_w3(i-1,:) = optiVel(i-1,:);
        
        % calc bias for accel
%         diffA(i,1:3) = a_body3 - (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)])';
%         differ = diffA(i,1:3);
        % buf_a = accel(:,:) - diff;
    end
  
    % METHOD1: Delftse  
    az1 = -9.81/(cos(theta) * cos(phi));
    % kdx1 = 0.5; kdy1 = 0.5;
    acc_w(i,1) = (cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi))*az1 - kdx1 * vel_w(i-1,1);
    acc_w(i,2) = (cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi))*az1 - kdy1 * vel_w(i-1,2);
    acc_w(i,3) =  cos(theta) * cos(phi) * az1 + 9.81;
    vel_w(i,1:3) = vel_w(i-1,1:3) + acc_w(i,1:3) .* dt;
    pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt; 
    
    
    % METHOD new: Coroilois time
    
    
    

    % METHOD2: Kumar's method
    % thrust = R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)]; 
    Vb = R * vel_w2(i-1, :)';
    Vb(3) = optiVel(i-1, 3);
    % Vb = R * optiVel(i-1, :)';
    Tnew(i) = [rms(rpm(i,:))^2, (mean(rpm(i,:))*Vb(3)), (Vb(1)^2 + Vb(2)^2)] * x;
    alpha = 1;
    compl_filter = alpha * accel(i,3) + (1-alpha) * Tnew(i);
    kd = rpmAvg(i,1) * [-kdx2 0 0; 0 -kdy2 0; 0 0 0];
    a_body = (kd * R * vel_w2(i-1, 1:3)')';
    acc_t = ([0;0;9.81] + R'* ([0;0;compl_filter] + [a_body(1); a_body(2); a_body(3)]))'; % a_body(3) = 0;
    
    acc_w2(i,1) = acc_t(1);
    acc_w2(i,2) = acc_t(2);
    acc_w2(i,3) = acc_t(3);
    vel_w2(i,1:3) = vel_w2(i-1,1:3) + (acc_w2(i,1:3) .* dt);
    pos_w2(i,1:3) = pos_w2(i-1,1:3) + (vel_w2(i,1:3) .* dt); %+ 0.5 .* acc_w2(i,1:3) .* dt .* dt; 
    
    % METHOD3: Tarek's method
    thrust(i,:) = (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)])';
    kd = thrust(i,3) * [-kdx3 0 0; 0 -kdy3 0; 0 0 0];
    a_body = (kd * R * vel_w3(i-1, 1:3)')'; % plus some bias. 
    
    % to mess up the results, vertical velocity does depend on lateral
    % movements, take that into account
    % newR = R';
    % newR(3,:) = [0 0 0];
    % innovation reduce   
    %a_body3 = accel(i,:) - differ;
    
    acc_t = ([0;0;9.81] + R'* [0;0;thrust(i,3)] + R' * [a_body(1); a_body(2); 0])';
    acc_w3(i,1) = acc_t(1);
    acc_w3(i,2) = acc_t(2);
    acc_w3(i,3) = acc_t(3);
    
%     entitya(i,1) = optiAcc(i,1) - acc_w3(i,1);
%     bodyv(i,:) = [R * optiVel(i,:)']';
    vel_w3(i,1:3) = vel_w3(i-1,1:3) + (acc_w3(i,1:3) .* dt);
    pos_w3(i,1:3) = pos_w3(i-1,1:3) + (vel_w3(i,1:3) .* dt);% + 0.5 .* acc_w3(i,1:3) .* dt .* dt;
      

end

%% verify filters


% plot the dead reckoning by two methods 
figure; 

plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3)); 
axis equal; hold on; grid on;
plot3(pos_w2(:,1), pos_w2(:,2), pos_w2(:,3));
plot3(pos_w3(:,1), pos_w3(:,2), pos_w3(:,3));
plot3(optiPos(:,1), optiPos(:,2), optiPos(:,3));
legend('filter1', 'filter2', 'filter3', 'gt'); 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

top = figure; 
plot(pos_w(:,1), pos_w(:,2)); 
axis equal; hold on; grid on;
plot(pos_w2(:,1), pos_w2(:,2));
plot(pos_w3(:,1), pos_w3(:,2));
plot(optiPos(:,1), optiPos(:,2));
legend('filter1', 'filter2', 'filter3', 'gt'); 
xlabel('x (m)'); ylabel('y (m)')
text(optiPos(1,1), optiPos(1,2), 'start');
text(optiPos(end,1), optiPos(end,2), 'end');

posPlot = figure;
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

velPlot = figure;
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

accPlot = figure;
sgtitle('acceleration');
subplot(3,1,1);
plot(t, optiAcc(:,1));
hold on; grid on;
plot(t, acc_w(:,1)); 
plot(t, acc_w2(:,1));
plot(t, acc_w3(:,1));
legend('gt', 'filter1', 'filter2', 'filter3'); xlabel('time (s)'); ylabel('a_x (m/s^2)');

subplot(3,1,2);
plot(t, optiAcc(:,2));
hold on; grid on;
plot(t, acc_w(:,2));
plot(t, acc_w2(:,2));
plot(t, acc_w3(:,2));
legend('gt','filter1', 'filter2', 'filter3'); xlabel('time (s)'); ylabel('a_y (m/s^2)');

subplot(3,1,3);
plot(t, optiAcc(:,3));
hold on; grid on;
plot(t, acc_w(:,3)); 
plot(t, acc_w2(:,3));
plot(t, acc_w3(:,3));
legend('gt','filter1', 'filter2', 'filter3'); xlabel('time (s)'); ylabel('a_z (m/s^2)');

%%
saveas(top, 'top', 'epsc');
saveas(posPlot, 'posPlot', 'epsc');
saveas(velPlot, 'velPlot', 'epsc');
saveas(accPlot, 'accPlot', 'epsc');
%%
% csvwrite('dragCo.csv', [kdx1, kdy1, kdx2, kdy2, kdx3, kdy3]);