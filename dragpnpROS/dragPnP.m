%% Take plots from ros, estimate drag co-efficients from PnP measurements 
% fly all directions for unbiased (no control over minimum variance observations)
clc;
close all;
clear all;


%%

filename = 'tf.csv';
A = csvread(filename, 1, 0);
gt_t   = A(:,1);
gtPos  = A(:,2:4);
gt_ang = A(:,5:7); 
gt_dt  = mean(gradient(gt_t)); % almost constant ros rate
gt_t   = gt_t - gt_t(1);

n = 150;
gtVel(:,1) = smooth(gradient(gtPos(:,1))/gt_dt, n);
gtVel(:,2) = smooth(gradient(gtPos(:,2))/gt_dt, n);
gtVel(:,3) = smooth(gradient(gtPos(:,3))/gt_dt, n);

gtAcc(:,1) = smooth(gradient(gtVel(:,1))/gt_dt, n);
gtAcc(:,2) = smooth(gradient(gtVel(:,2))/gt_dt, n);
gtAcc(:,3) = smooth(gradient(gtVel(:,3))/gt_dt, n);

filename = 'imu.csv';
C = csvread(filename, 1, 0);
accel = C(:,2:4);

%%

for idx = 2:1:length(gt_t)
    gtVel(idx,1) = (gtPos(idx,1) - gtPos(idx-1, 1))/(gt_t(idx) - gt_t(idx-1));
    gtVel(idx,2) = (gtPos(idx,2) - gtPos(idx-1, 2))/(gt_t(idx) - gt_t(idx-1));
    gtVel(idx,3) = (gtPos(idx,3) - gtPos(idx-1, 3))/(gt_t(idx) - gt_t(idx-1));
end

gtVel(:,1) = smooth(gtVel(:,1), 10);
gtVel(:,2) = smooth(gtVel(:,2), 10);
gtVel(:,3) = smooth(gtVel(:,3), 10);

for idx = 2:1:length(gt_t)
    gtAcc(idx,1) = (gtVel(idx,1) - gtVel(idx-1, 1))/(gt_t(idx) - gt_t(idx-1));
    gtAcc(idx,2) = (gtVel(idx,2) - gtVel(idx-1, 2))/(gt_t(idx) - gt_t(idx-1));
    gtAcc(idx,3) = (gtVel(idx,3) - gtVel(idx-1, 3))/(gt_t(idx) - gt_t(idx-1));
end

gtAcc(:,1) = smooth(gtAcc(:,1), 10);
gtAcc(:,2) = smooth(gtAcc(:,2), 10);
gtAcc(:,3) = smooth(gtAcc(:,3), 10);



%% 

bodyAcc = zeros(length(gt_t), 3);
bodyVel = zeros(length(gt_t), 3);

for i = 1:1:length(gt_t)
    
    phi   = gt_ang(i,1);
    theta = gt_ang(i,2);
    psi   = gt_ang(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
    %bodyAcc_t = R * [gtAcc(i,1); gtAcc(i,2); (gtAcc(i,3) + 9.81)]; % add grav here NED
    bodyAcc_t = accel(i,1:3);
    bodyVel_t = R * [gtVel(i,1); gtVel(i,2); gtVel(i,3)];

    bodyAcc(i,1) = bodyAcc_t(1);
    bodyVel(i,1) = bodyVel_t(1);

    bodyAcc(i,2) = bodyAcc_t(2);
    bodyVel(i,2) = bodyVel_t(2);

    bodyAcc(i,3) = bodyAcc_t(3);
    bodyVel(i,3) = bodyVel_t(3);

end

%%

% plot(gt_t, bodyAcc(:,1)); hold on;
% plot(gt_t, accelero(:,1));




%% linear drag model, simple

figure; 
subplot(2,1,1);
plot(bodyVel(:,1), bodyAcc(:,1), '.'); 
hold on; grid on;
p_x = polyfit(bodyVel(:,1), bodyAcc(:,1), 1);
yaxis = polyval(p_x, bodyVel(:,1));
plot(bodyVel(:,1), yaxis, 'LineWidth', 2);
kdx1 = -p_x(1);
xlabel('vel^B_x (m/s)'); ylabel('acc^B_x (m/s^2)');
title(['ground truth drag kd_x: ', num2str(kdx1)]);

subplot(2,1,2); 
plot(bodyVel(:,2), bodyAcc(:,2), '.'); 
hold on; grid on;
p_y = polyfit(bodyVel(:,2), bodyAcc(:,2), 1);
yaxis = polyval(p_y, bodyVel(:,2));
plot(bodyVel(:,2), yaxis, 'LineWidth', 2);
xlabel('vel^B_y (m/s)'); ylabel('acc^B_y (m/s^2)');
kdy1 = -p_y(1);
title(['ground truth drag kd_y: ', num2str(kdy1)]);



%% now with PnP

filename = 'pnp_f.csv'; 
B      = csvread(filename, 1, 0);
pnpt   = B(:,1);
pnpPos = B(:,2:4);
pnpAng = B(:,5:7); % find out if these are from optitrack or not
accelero = B(:,8:10);
% pnpt = pnpt - A(1,1);

% lsim not possible, requires equal time intervals
% filter_acc = mkfilter(2, 5, 'bessel');
% filt_a(:,1) = lsim(filter_acc, accelero(:,1), pnpt);
% filt_a(:,2) = lsim(filter_acc, accelero(:,2), pnpt);
% filt_a(:,3) = lsim(filter_acc, accelero(:,3), pnpt);






%% kick out multiple detections with dt = 0
pnpt = pnpt - A(1,1);
dt_pnp = gradient(pnpt);
dup_idx = find(dt_pnp == 0);
pnpt(dup_idx) = [];
pnpPos(dup_idx, :) = [];
pnpAng(dup_idx, :) = [];
accelero(dup_idx, :) = [];
dt_pnp(dup_idx) = [];
pnpVel = zeros(length(pnpt), 3);
pnpAcc = zeros(length(pnpt), 3);


%% 


%%
figure;
plot(pnpPos(:,1), pnpPos(:,2), 'xr'); hold on;
plot(gtPos(:,1), gtPos(:,2)); hold on; axis equal; grid on;
xlabel('x'); ylabel('y');
text(gtPos(1,1), gtPos(1,2), 'start');
text(gtPos(end,1), gtPos(end,2), 'end');

%%

for idx = 2:1:length(pnpt)-1    
    pnpVel(idx,1) = (pnpPos(idx,1) - pnpPos(idx-1, 1))/dt_pnp(idx);
    pnpVel(idx,2) = (pnpPos(idx,2) - pnpPos(idx-1, 2))/dt_pnp(idx);
    pnpVel(idx,3) = (pnpPos(idx,3) - pnpPos(idx-1, 3))/dt_pnp(idx);
   
    % kick out nans (detection with same time stamp, why would that happen)
%     pnpVel(isnan(pnpVel)) = 0;
%     pnpVel(isinf(pnpVel)) = 0;
end

pnpVel(:,1) = smooth(pnpVel(:,1), 40);
pnpVel(:,2) = smooth(pnpVel(:,2), 40);
pnpVel(:,3) = smooth(pnpVel(:,3), 40);

% pnpVel(isnan(pnpVel)) = 0;
% pnpVel(isinf(pnpVel)) = 0;

for idx = 2:1:length(pnpt)
    pnpAcc(idx,1) = (pnpVel(idx,1) - pnpVel(idx-1, 1))/(pnpt(idx) - pnpt(idx-1)); % dt_pnp(idx)
    pnpAcc(idx,2) = (pnpVel(idx,2) - pnpVel(idx-1, 2))/(pnpt(idx) - pnpt(idx-1)); % dt_pnp(idx)
    pnpAcc(idx,3) = (pnpVel(idx,3) - pnpVel(idx-1, 3))/(pnpt(idx) - pnpt(idx-1)); % dt_pnp(idx)
    
%     pnpAcc(isnan(pnpAcc)) = 0;
%     pnpAcc(isinf(pnpAcc)) = 0;
end

% pnpAcc(:,1) = smooth(pnpAcc(:,1), 10);
% pnpAcc(:,2) = smooth(pnpAcc(:,2), 10);
% pnpAcc(:,3) = smooth(pnpAcc(:,3), 10);

% pnpAcc(isnan(pnpAcc)) = 0;
% pnpAcc(isinf(pnpAcc)) = 0;


%% 
% figure;
% plot(pnpt, pnpVel(:,1), '.r'); hold on;
% plot(gt_t, gtVel(:,1));
% figure;
% plot(pnpt, pnpAcc(:,1), '.r'); hold on;
% plot(gt_t, gtAcc(:,1));

%%

pnpbodyAcc = zeros(length(pnpt), 3);
pnpbodyVel = zeros(length(pnpt), 3);
bodyVel_t = zeros(1,3);
for i = 2:1:length(pnpt)
    
    phi   = pnpAng(i,1);
    theta = pnpAng(i,2);
    psi   = pnpAng(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
  
    bodyAcc_t = R * [pnpAcc(i,1); pnpAcc(i,2); (pnpAcc(i,3) + 9.81)]; % add grav here NED
    bodyAcc_t = accelero(i,:);
    bodyVel_t = R * [pnpVel(i,1); pnpVel(i,2); pnpVel(i,3)];

    pnpbodyAcc(i,1) = bodyAcc_t(1);
    pnpbodyVel(i,1) = bodyVel_t(1);

    pnpbodyAcc(i,2) = bodyAcc_t(2);
    pnpbodyVel(i,2) = bodyVel_t(2);

    pnpbodyAcc(i,3) = bodyAcc_t(3);
    pnpbodyVel(i,3) = bodyVel_t(3);

end
%%
% 
% plot(pnpt, pnpbodyAcc(:,1)); hold on;
% plot(pnpt, accelero(:,1));



%%
figure; 
subplot(2,1,1);
plot(pnpbodyVel(:,1), pnpbodyAcc(:,1), '.'); 
hold on; grid on;
p_x = polyfit(pnpbodyVel(:,1), pnpbodyAcc(:,1), 1);
yaxis = polyval(p_x, pnpbodyVel(:,1));
plot(pnpbodyVel(:,1), yaxis, 'LineWidth', 2);
xlabel('vel^B_x (m/s)'); ylabel('acc^B_x (m/s^2)');
kdx1 = -p_x(1);
title(['PnP drag kd_x: ', num2str(kdx1)]);

subplot(2,1,2);
plot(pnpbodyVel(:,2), pnpbodyAcc(:,2), '.'); 
hold on; grid on;
p_y = polyfit(pnpbodyVel(:,2), pnpbodyAcc(:,2), 1);
yaxis = polyval(p_y, pnpbodyVel(:,2));
plot(pnpbodyVel(:,2), yaxis, 'LineWidth', 2);
xlabel('vel^B_y (m/s)'); ylabel('acc^B_y (m/s^2)');
kdy1 = -p_y(1);
title(['PnP drag kd_y: ', num2str(kdy1)]);


%%
% filename = 'draglog.csv';  % plot(optiPos(:,1), optiPos(:,2)); axis equal, log of straight traj
% % ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
% M = csvread(filename, 1, 0);
% col = size(M,2);
% 
% fid = fopen(filename);
% a = textscan(fid,'%s',1);
% C = strsplit(string(a),',');
% fclose(fid);
% 
% filtert = M(:,1);
% 
% g =  9.81;
% pnpAng = M(:,5:7); % find out if these are from optitrack or not
% filterPos = M(:,2:4); % find out if using optiTrack height or not



