%% state estimation main


% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;

filename = '../logs/2019-07-03_13_26_13.csv';
M = csvread(filename, 1, 0);
col = size(M,2);

accel = M(:,2:4)./1024;  % INT32_ACCEL_FRAC
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
st = 40;
[filt_a, optiAcc, optiVel]= optiData(optiPos, accel, t, dt, 150, st);

% or read from old dataset to prevent overfitting
rpmAvg = mean(rpm,2);
[kdx1, kdy1, kdx2, kdy2, kdx3, kdy3] = readDragCo();

%% compensate before yawing

grndtruth = zeros(length(t), 3);
for i = 1:1:length(t)
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    grndtruth(i,:) = (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)])';
end

[comp_acc, parx, pary, parz] = compensator(grndtruth, accel, t);

%% 

st = 40;
acc_w = zeros(length(t), 3);
vel_w = zeros(length(t), 3);
pos_w = zeros(length(t), 3);

acc_w2 = zeros(length(t), 3);
vel_w2 = zeros(length(t), 3);
pos_w2 = zeros(length(t), 3);

acc_w3 = zeros(length(t), 3);
vel_w3 = zeros(length(t), 3);
pos_w3 = zeros(length(t), 3);

pos_w(1:st,:) = optiPos(1:st,:);
vel_w(1:st,:) = optiVel(1:st,:);

pos_w2(1:st,:) = optiPos(1:st, :);
vel_w2(1:st,:) = optiVel(1:st,:);

pos_w3(1:st,:) = optiPos(1:st,:);
vel_w3(1:st,:) = optiVel(1:st,:);

thrust = zeros(length(t), 3);

Vb = zeros(length(t), 3);
Vbz = zeros(length(t), 1);
Vh = zeros(length(t), 1);
newT = zeros(length(t), 1);

bodyVel = zeros(length(t), 3);
avgRpm = zeros(length(t), 1);

for i = st:1:length(t)
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3) - 0.5/2.5;
    
    % this is world to body
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
      % emulated PnP
    % update the positions and velocities atleast 20 times
    if rem(i,round(length(t)/20)) == 0 
%         pos_w(i:i-1,:)  = optiPos(i:i-1,:);
%         pos_w2(i:i-1,:) = optiPos(i:i-1,:);
%         pos_w3(i:i-1,:) = optiPos(i:i-1,:);
%         
%         vel_w(i:i-1,:)  = optiVel(i:i-1,:);
%         vel_w2(i:i-1,:) = optiVel(i:i-1,:);
%         vel_w3(i:i-1,:) = optiVel(i:i-1,:);
        
    end
  
    
    % METHOD new: Coriolis time
    % someran = cross(rateBody(i,1:3)' , (R * optiVel(i, 1:3)'));
    % - 0.5 * rssq(R * [optiVel(i-1,1); optiVel(i-1,2); 0]); % + someran(3); %
    
    % METHOD1: Delftse
    az1 = -9.81/(cos(theta * 0.8) * cos(phi* 0.8)); 
    % thrust(i,:) = (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)])';
    % az1 = thrust(i,3);
    acc_w(i,1) = (cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi))*az1 - kdx1 * vel_w(i-1,1);
    acc_w(i,2) = (cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi))*az1 - kdy1 * vel_w(i-1,2);
    acc_w(i,3) =  cos(theta) * cos(phi) * az1 + 9.81;
    vel_w(i,1:3) = vel_w(i-1,1:3) + acc_w(i,1:3) .* dt;
    pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt; 
    
    % METHOD2: Kumar
    bodyVel = (R * [optiVel(i,1); optiVel(i,2); optiVel(i,3)]);
    kd = rpmAvg(i,1) * [-kdx2 0 0; 0 -kdy2 0; 0 0 0]; % depends on x??!
    a_body = (kd * R * vel_w2(i-1, 1:3)')';    
    thr_axis = (-9.81/(cos(theta * 0.8) * cos(phi * 0.8))); 
    acc_t = ([0;0;9.81] + R'* ([0;0; thr_axis] + [a_body(1); a_body(2); a_body(3)]))';
    acc_w2(i,1) = acc_t(1);
    acc_w2(i,2) = acc_t(2);
    acc_w2(i,3) = acc_t(3);
    vel_w2(i,1:3) = vel_w2(i-1,1:3) + (acc_w2(i,1:3) .* dt);
    pos_w2(i,1:3) = pos_w2(i-1,1:3) + (vel_w2(i,1:3) .* dt);
    
    % METHOD3: Tarek's method
    thrust(i,:) = (R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)])';
    kd = thrust(i,3) * [-kdx3 0 0; 0 -kdy3 0; 0 0 0];
    a_body = (kd * R * vel_w3(i-1, 1:3)')'; % plus some bias. 
    a_body(3) = 0;
    acc_t = ([0;0;9.81] + R'* [0;0;thrust(i,3)] + R' * [a_body(1); a_body(2); a_body(3)])';
    acc_w3(i,1) = acc_t(1);
    acc_w3(i,2) = acc_t(2);
    acc_w3(i,3) = acc_t(3);
    
    vel_w3(i,1:3) = vel_w3(i-1,1:3) + (acc_w3(i,1:3) .* dt);
    pos_w3(i,1:3) = pos_w3(i-1,1:3) + (vel_w3(i,1:3) .* dt);
    
    bodyVel(i,1:3) = (R * optiVel(i,:)')';

end

 %% verify filters


% plot the dead reckoning by two methods 
threed = figure; 
set(threed,'DefaultLineLineWidth',2)

plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3)); 
axis equal; hold on; grid on;
plot3(pos_w2(:,1), pos_w2(:,2), pos_w2(:,3));
plot3(pos_w3(:,1), pos_w3(:,2), pos_w3(:,3));
plot3(optiPos(:,1), optiPos(:,2), optiPos(:,3));
legend('Delft', 'Kumar', 'Tarek', 'gt'); 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

top = figure; 
plot(pos_w(:,1), pos_w(:,2)); 
axis equal; hold on; grid on;
plot(pos_w2(:,1), pos_w2(:,2));
plot(pos_w3(:,1), pos_w3(:,2));
plot(optiPos(:,1), optiPos(:,2));
legend('Delft', 'Kumar', 'Tarek', 'gt');  
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
legend('Delft', 'Kumar', 'Tarek', 'gt');  xlabel('time (s)'); ylabel('x (m)');

subplot(3,1,2);
plot(t, pos_w(:,2)); 
hold on; grid on; 
plot(t, pos_w2(:,2));
plot(t, pos_w3(:,2));
plot(t, optiPos(:,2));
legend('Delft', 'Kumar', 'Tarek', 'gt');  xlabel('time (s)'); ylabel('y (m)');

subplot(3,1,3);
plot(t, pos_w(:,3)); 
hold on; grid on; 
plot(t, pos_w2(:,3));
plot(t, pos_w3(:,3));
plot(t, optiPos(:,3));
legend('Delft', 'Kumar', 'Tarek', 'gt');  xlabel('time (s)'); ylabel('z (m)');

velPlot = figure;
sgtitle('lateral velocity');
subplot(3,1,1);
plot(t, vel_w(:,1)); 
hold on; grid on; 
plot(t, vel_w2(:,1));
plot(t, vel_w3(:,1));
plot(t, optiVel(:,1));
legend('Delft', 'Kumar', 'Tarek', 'gt');  xlabel('time (s)'); ylabel('v_x (m/s)');

subplot(3,1,2);
plot(t, vel_w(:,2)); 
hold on; grid on; 
plot(t, vel_w2(:,2));
plot(t, vel_w3(:,2));
plot(t, optiVel(:,2));
legend('Delft', 'Kumar', 'Tarek', 'gt');  xlabel('time (s)'); ylabel('v_y (m/s)');

subplot(3,1,3);
plot(t, vel_w(:,3)); 
hold on; grid on; 
plot(t, vel_w2(:,3));
plot(t, vel_w3(:,3));
plot(t, optiVel(:,3));
legend('Delft', 'Kumar', 'Tarek', 'gt');  xlabel('time (s)'); ylabel('v_z (m/s)');

accPlot = figure;
sgtitle('acceleration');
subplot(3,1,1);
plot(t, optiAcc(:,1));
hold on; grid on;
plot(t, acc_w(:,1)); 
plot(t, acc_w2(:,1));
plot(t, acc_w3(:,1));
legend('gt', 'Delft', 'Kumar', 'Tarek'); xlabel('time (s)'); ylabel('a_x (m/s^2)');

subplot(3,1,2);
plot(t, optiAcc(:,2));
hold on; grid on;
plot(t, acc_w(:,2));
plot(t, acc_w2(:,2));
plot(t, acc_w3(:,2));
legend('gt','Delft', 'Kumar', 'Tarek'); xlabel('time (s)'); ylabel('a_y (m/s^2)');

subplot(3,1,3);
plot(t, optiAcc(:,3));
hold on; grid on;
plot(t, acc_w(:,3)); 
plot(t, acc_w2(:,3));
plot(t, acc_w3(:,3));
legend('gt','Delft', 'Kumar', 'Tarek'); xlabel('time (s)'); ylabel('a_z (m/s^2)');

%%
saveas(top, 'top', 'epsc');
saveas(posPlot, 'posPlot', 'epsc');
saveas(velPlot, 'velPlot', 'epsc');
saveas(accPlot, 'accPlot', 'epsc');




%%
% csvwrite('dragCo.csv', [kdx1, kdy1, kdx2, kdy2, kdx3, kdy3]);

%% KUMAR ORIGINAL 


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
    


%% check rate body
ang1= zeros(length(t),1); ang2 = zeros(length(t),1); ang3 = zeros(length(t),1);
ang3(1,1) = angBody(1,3);
for i = 2:1:length(t)
    ang1(i,1) = ang1(i-1,1) + rateBody(i-1,1) * dt;
    ang2(i,1) = ang2(i-1,1) + rateBody(i-1,2) * dt;
    ang3(i,1) = ang3(i-1,1) + rateBody(i-1,3) * dt;
end
    
figure;
plot(t, ang3); hold on;
% plot(t, ang2);
% plot(t, ang3);
plot(t, angBody(:,3));


% do you want to yaw a bit?  Because optiTrack yaw is blah?

%% 

plot(t, optiPos(:,1))
plot(t, angBody(:,1) * 180/pi); hold on;
plot(t, angBody(:,1)); hold on;
plot(t, optiPos(:,1)); hold on;
plot(t, angBody(:,2))
plot(t, optiPos(:,2))
