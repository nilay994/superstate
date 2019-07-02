%% TO find the thrust and convergent VBZ and z position estimates
% two methods: use accelerometer non causal or not

% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;
% what worked for drag est, fits well for thrust model, also mean of acceleration is good
% filename = '../logs/2019-06-24_14_29_46.csv'; 
filename = '/home/nilay/Downloads/2019-07-02_13_20_10.csv';
M = csvread(filename, 1, 0);
col = size(M,2);
% M = M(40000:50000, :);
accel = M(:,2:4)./1024;  % INT32_ACCEL_FRAC
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

dr_state.x = M(:,25);
dr_state.y = M(:,26);

dr_cmd.roll  = M(:,27);
dr_cmd.pitch = M(:,28);

[kdx1, kdy1, kdx2, kdy2, kdx3, kdy3] = readDragCo();

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

% somevar(:,3) = lsim(filter_acc, optiAcc(:,3), t);

%% check if thrust matches (in case of incorrect altitude pprz) 
T = thrustMatch(angBody, optiAcc, filt_a, t);


% nullify acc bias
compensated_acc = zeros(length(t), 3);
% for body z
C = zeros(length(t),3);
z_arr = zeros(length(t),1);
st = 1;
for i =  st:1:length(t)
    C(i,:) = [1, t(i), t(i)^2];
    z_arr(i,1) = T(i) - accel(i,3);
end
par_z = C\z_arr


% reproject z
for i =  st:1:length(t)
    error = C(i,:) * par_z;
    compensated_acc(i,3) = accel(i,3) + error;
end

figure;
plot(t, T); hold on;
plot(t, accel(:,3));
plot(t, compensated_acc(:,3));
filt_a(:,3) = lsim(filter_acc, compensated_acc(:,3), t);


% y = abs(T)./max(max(abs(T)));
%% least squares to the rescue in progress thrust predictor

A = zeros(length(t), 3);

Vb  = zeros(length(t), 3);
Vbz = zeros(length(t), 1);
Vh  = zeros(length(t), 1);


avgRpmSq   = zeros(length(t), 1);
avgRpmMean = zeros(length(t), 1);

% minimize cost function, thanks to linear models
for i=1:1:length(t)
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
    Vb(i,1:3) = R * optiVel(i,:)';
    Vbz(i) = Vb(i,3); 
    Vh(i,1) = (Vb(i,1)^2 + Vb(i,2)^2); 
    avgRpmSq(i) = (sum(rpm(i,:))/4)^2;
    avgRpmMean(i) = sum(rpm(i,:)/4);
    A(i, 1:3) = [avgRpmSq(i), ...
        avgRpmMean(i) * Vbz(i), ...
        Vh(i,1)];  %accel(i,3) filt_a(i,3)
end

%% least sqaures time :) 
x = A\T

%%
% forward modelling
newT = zeros(length(t), 1);

%idx = max(find(t<6)):1:min(find(t>40));
idx = 1:1:length(t);
for i=idx
    
    phi = angBody(i,1);
    theta = angBody(i,2);
    psi = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
    Vb = R * optiVel(i,:)';
    
    newT(i,1) = [avgRpmSq(i), ...
        - avgRpmMean(i) * Vbz(i), ...
        - Vh(i,1)] * x;  % , filt_a(i,3)

% doesn't work yet, bias should have less weight in the fit?
%     newT(i,1) = [1,...
%         rms(rpm(i,:))^2, ...
%         (mean(rpm(i,:))*Vb(3)), ...
%         (Vb(1)^2 + Vb(2)^2)] * x;
    
end

% not sure why fit takes time to converge
idx = 1:t(end); %find(t>6 & t<50);
% eps = T(idx)-newT(idx);
eps = T - newT;
sprintf("rms of fit, less is good: %f", rms(eps))

% gaussian fits only in case of cropped flight data using filter and not
% smooth while deriving optiTrack accelerations
figure;  
histogram(eps, 'Normalization', 'pdf'); 
title("rms of thrust model fit without accmeter (causal) is: " + num2str(rms(eps)));
%% 
figure;
oldActual = T;
newRemap = newT;
plot(t, oldActual); hold on; grid on;
plot(t, newRemap);
% plot(t, filt_a(:,3));
legend('oldActual', 'newRemap');
title('compare fit');


%% 

st = 200;

newThr = zeros(length(t), 3);
oldThr = zeros(length(t), 3);

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
acc_w(1:st,:) = optiAcc(1:st,:);

newnewT = zeros(length(t), 1);
newnewT(1:st,1) = optiAcc(1:st, 3);

% avgRpmSq = zeros(length(t), 1);
% avgRpmMean =  zeros(length(t), 1);

for i = st-1:1:length(t)
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);%  - 0.7/3.5;
    
    if rem(i,round(length(t)/5)) == 0 
%         pos_w(i-1:i,:)  = optiPos(i-1:i,:);        
%         vel_w(i-1:i,:)  = optiVel(i-1:i,:); 
        
    end
    
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];

    avgRpmSq(i) = (sum(rpm(i,:))/4)^2;
    avgRpmMean(i) = sum(rpm(i,:)/4);
  
    % bodyVel = (R * [optiVel(i,1); optiVel(i,2); optiVel(i,3)]);
    kd = avgRpmMean(i,1) * [-kdx2 0 0; 0 -kdy2 0; 0 0 -x(2)]; 
     % Vbz(i) = Vb(i,3);
    a_body = (kd * R * vel_w(i-1, 1:3)');
       
    % Vb(i,1:3) = R * optiVel(i,:)';
    Vb(i,1:3) = R * vel_w(i-1, 1:3)';
    Vh(i,1) = (Vb(i,1)^2 + Vb(i,2)^2); 
   
%     newT(i,1) = [avgRpmSq(i), ...
%     avgRpmMean(i) * Vbz(i), ...
%     Vh(i,1), filt_a(i,3)] * x;

    % x(2) is already accounted for in a_body vector
    %newnewT(i,1) = [avgRpmSq(i), Vh(i,1), filt_a(i,3)] * [x(1)*1.05; x(3)/2; x(4)];
    newnewT(i,1) = [avgRpmSq(i), Vh(i,1)] * [x(1); x(3)];
    % this signifies thrust increases while descending and decreases while
    % going forward? Which is untrue??
    % thr_axis = alpha * newnewT(i,1) + (1-alpha) * (-9.81/(cos(theta * 0.85) * cos(phi * 0.85))); 
    thr_axis = newnewT(i,1);
    acc_t = ([0;0;9.81] + R'* ([0;0; thr_axis] + [a_body(1); a_body(2); a_body(3)]))';
    
    acc_w(i,1) = acc_t(1);
    acc_w(i,2) = acc_t(2);
    acc_w(i,3) = acc_t(3);
    vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w(i,1:3) .* dt);
    pos_w(i,1:3) = pos_w(i-1,1:3) + (vel_w(i,1:3) .* dt); 
    
end


 %% verify filters


% plot the dead reckoning by two methods 
figure; 

plot3(pos_w(:,1), pos_w(:,2), pos_w(:,3)); 
axis equal; hold on; grid on;
plot3(optiPos(:,1), optiPos(:,2), optiPos(:,3));
legend('estimator', 'gt'); 
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

top = figure; 
plot(pos_w(:,1), pos_w(:,2)); 
axis equal; hold on; grid on;
plot(optiPos(:,1), optiPos(:,2));
legend('estimator', 'gt'); 
xlabel('x (m)'); ylabel('y (m)')
text(optiPos(1,1), optiPos(1,2), 'start');
text(optiPos(end,1), optiPos(end,2), 'end');

posPlot = figure('Position', get(0, 'Screensize'));
sgtitle('position');
subplot(3,1,1);
plot(t, pos_w(:,1)); 
hold on; grid on; 
plot(t, optiPos(:,1));
legend('estimator','gt'); xlabel('time (s)'); ylabel('x (m)');

subplot(3,1,2);
plot(t, pos_w(:,2)); 
hold on; grid on; 
plot(t, optiPos(:,2));
legend('estimator', 'gt'); xlabel('time (s)'); ylabel('y (m)');

subplot(3,1,3);
plot(t, pos_w(:,3)); 
hold on; grid on; 
plot(t, optiPos(:,3));
legend('estimator','gt'); xlabel('time (s)'); ylabel('z (m)');

velPlot = figure('Position', get(0, 'Screensize'));
sgtitle('lateral velocity');
subplot(3,1,1);
plot(t, vel_w(:,1)); 
hold on; grid on; 
plot(t, optiVel(:,1));
legend('estimator', 'gt'); xlabel('time (s)'); ylabel('v_x (m/s)');

subplot(3,1,2);
plot(t, vel_w(:,2)); 
hold on; grid on; 
plot(t, optiVel(:,2));
legend('estimator', 'gt'); xlabel('time (s)'); ylabel('v_y (m/s)');

subplot(3,1,3);
plot(t, vel_w(:,3)); 
hold on; grid on; 
plot(t, optiVel(:,3));
legend('estimator', 'gt'); xlabel('time (s)'); ylabel('v_z (m/s)');

accPlot = figure('Position', get(0, 'Screensize'));
sgtitle('acceleration');
subplot(3,1,1);
plot(t, optiAcc(:,1));
hold on; grid on;
plot(t, acc_w(:,1)); 
legend('gt', 'estimator'); xlabel('time (s)'); ylabel('a_x (m/s^2)');

subplot(3,1,2);
plot(t, optiAcc(:,2));
hold on; grid on;
plot(t, acc_w(:,2));
legend('gt','estimator'); xlabel('time (s)'); ylabel('a_y (m/s^2)');

subplot(3,1,3);
plot(t, optiAcc(:,3));
hold on; grid on;
plot(t, acc_w(:,3)); 
legend('gt', 'estimator'); xlabel('time (s)'); ylabel('a_z (m/s^2)');

% saveas(posPlot, 'posPlot', 'epsc');
% saveas(velPlot, 'velPlot', 'epsc');
% saveas(accPlot, 'accPlot', 'epsc');

% a_body = alpha * a_body + (1 - alpha) * comp_acc(i,:); 
    % thr_axis = newT(i,1); 
    % thr_axis = alpha * (-9.81/((cos(theta)  * cos(phi))) + 0.5 * bodyVel(3)) + (1-alpha) * comp_acc(i,3);
    % do a simple compl between hover and kumar thrust model
    % thr_axis = -9.81/(cos(theta) * cos(phi)) + 0.5 * bodyVel(3);
    % thr_axis = -9.81/(cos(theta) * cos(phi)) + 0.15 * rssq(R * [optiVel(i,1); optiVel(i,2); 0]);
    