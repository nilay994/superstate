%% TO plot thrust to check vortices

% Step1: PARSE FLIGHT DATA
clc;
close all;
clear all;
filename = '2019-05-22_20_59_15.csv';  % plot(optiPos(:,1), optiPos(:,2)); axis equal, log of straight traj
% ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
M = csvread(filename, 1, 0);
col = size(M,2);
M = M(1:end-1, :);
fid = fopen(filename);
a = textscan(fid,'%s',1);
C = strsplit(string(a),',');
fclose(fid);

accel = M(:,2:4)./1024;
gyro  = M(:,5:7)./1024;
angBody = M(:,8:10); % find out if these are from optitrack or not %rpy
optiPos = M(:,11:13); % find out if using optiTrack height or not
cmd_thrust = M(:,14);
cmd_thrust = cmd_thrust./max(max(cmd_thrust));
T = cmd_thrust - 0.4;
T = T./max(max(T));
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

%%
st = 1;
[filt_a, optiAcc, optiVel]= optiData(optiPos, accel, t, dt, 150, st);
% plotEverything(angBody, optiPos, optiVel, optiAcc, t);
%%
aH = axes;

lH(1) = plot(aH,optiPos(:,1), -optiPos(:,3));%, 10, optiVel(:,1));

xlabel('x'); ylabel('z'); 
hold on; grid on;
axis equal;
set(lH,'hittest','off'); % so you can click on the Markers
hold on; 
set(aH,'ButtonDownFcn',{@getCoord,angBody,T}); % Defining what happens when clicking

figure;
x = optiPos(:,1);
y = -optiPos(:,3);
z = optiVel(:,1);
% Plot data:
surf([x(:) x(:)], [y(:) y(:)], [z(:) z(:)], ...  % Reshape and replicate data
     'FaceColor', 'none', ...    % Don't bother filling faces with color
     'EdgeColor', 'interp', ...  % Use interpolated color for edges
     'LineWidth', 2);            % Make a thicker line
xlim([-2.5, 0.5]);
ylim([0 3]);
xlabel('x'); ylabel('z'); 
view(2);   % Default 2-D view
h = colorbar;
ylabel(h, 'v_x (m/s)')


%%

figure;
x = optiPos(:,1);
y = -optiPos(:,3);

for i=1:1:length(optiVel(:,1))
    
    
    phi   = angBody(i,1);
    theta = angBody(i,2);
    psi   = angBody(i,3);
    
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    bodyVel(i,:) = (R * optiVel(i,:)')';
end
z = bodyVel(:,1);
% Plot data:
surf([x(:) x(:)], [y(:) y(:)], [z(:) z(:)], ...  % Reshape and replicate data
     'FaceColor', 'none', ...    % Don't bother filling faces with color
     'EdgeColor', 'interp', ...  % Use interpolated color for edges
     'LineWidth', 2);  
%% 

figure;
x = optiPos(:,1);
y = -optiPos(:,3);
z = optiAcc(:,1);
% Plot data:
surf([x(:) x(:)], [y(:) y(:)], [z(:) z(:)], ...  % Reshape and replicate data
     'FaceColor', 'none', ...    % Don't bother filling faces with color
     'EdgeColor', 'interp', ...  % Use interpolated color for edges
     'LineWidth', 2);            % Make a thicker line

xlabel('x'); ylabel('z'); 
view(2);   % Default 2-D view
h = colorbar;
ylabel(h, 'v_x (m/s)')