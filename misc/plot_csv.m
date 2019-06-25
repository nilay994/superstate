%% TO find the psuedoaccelerations on body fixed frame
% pending: yaw in the rotation matrix is it correct
% my hypothesis is it correct? 
%

close all;
filename = '2019-03-07_10_50_06.csv';
% ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
M = csvread(filename, 1, 0);
col = size(M,2);

fid = fopen(filename);
a = textscan(fid,'%s',1);
C = strsplit(string(a),',');
fclose(fid);

accel = M(:,2:4)/.4096;
gyro  = M(:,5:7)/.4096;
mag   = M(:,8:10);
angBody = M(:,11:13);
optiPos = M(:,14:16);
t = M(:,end);

for i=1:1:col
     figure('Name', C(i));
     plot(M(:,i));
end
%% 
figure(1);

subplot(3,1,1);
plot(t, M(:,11)*180/pi);
title('phi'); ylim([-90 90]);
grid on; 

subplot(3,1,2);
plot(t, M(:,12)*180/pi);
title('theta'); ylim([-90 90]);
grid on;

subplot(3,1,3);
plot(t, M(:,13)*180/pi);
title('psi'); ylim([-90 90]);
grid on;
%% preprocess (RUN ONCE ONLY!!)

% invert optitrack z axis or does it require rotation matrix?
optiPos(:,end) = -1 * optiPos(:,end); 

% plot the original data
figure('Name', "rot");
ax1 = subplot(2,1,1);
scatter3(optiPos(:,1), optiPos(:,2), optiPos(:,3), '.b');
text(optiPos(1,1), optiPos(1,2), optiPos(1,3), 'start');
xlabel('x'); ylabel('y'); zlabel('z');
xlim([-3 3]); ylim([-3 3]); zlim([0 3]);
axis square; grid on;

% my rotation mat = -ve for acw, match optitrack with NED
d =  32.5*3.142/180;
orient = [cos(d) sin(d); -sin(d) cos(d)];
optiPos(:,1) =  cos(d)*optiPos(:,1) + sin(d)*optiPos(:,2);
optiPos(:,2) = -sin(d)*optiPos(:,1) + cos(d)*optiPos(:,2);

% plot the rotated data
ax2 = subplot(2,1,2);
scatter3(optiPos(:,1), optiPos(:,2), optiPos(:,3), '.r');
xlabel('x'); ylabel('y'); zlabel('z');
text(optiPos(1,1), optiPos(1,2), optiPos(1,3), 'start');
xlim([-3 3]); ylim([-3 3]); zlim([0 3]);
axis square; grid on;

Link = linkprop([ax1, ax2],{'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', Link);


%% plot 3d traj over time

startSim = M(1,1)/512;
endSim = M(1,end)/512;
stepsize = 1/512; % data logging freq

figure('Name',"traj");
cnt = 1;
curve = animatedline('LineWidth',2);
view(43,24); hold on; grid on;
for t = startSim:stepsize:endSim
    addpoints(curve, optiPos(cnt,1), optiPos(cnt,2), optiPos(cnt,3));
    pause(stepsize)
    cnt = cnt+1;
    drawnow
end

%% plot opti x, xd, xdd
% todo: verify imu conventions
dt = 1/512;
g = 9.81;

optiVel(:,1) = smooth(gradient(optiPos(:,1))/dt, 150);
optiVel(:,2) = smooth(gradient(optiPos(:,2))/dt, 150);
optiVel(:,3) = smooth(gradient(optiPos(:,3))/dt, 150);

optiAcc(:,1) = smooth(gradient(optiVel(:,1))/dt, 150);
optiAcc(:,2) = smooth(gradient(optiVel(:,2))/dt, 150);
optiAcc(:,3) = smooth(gradient(optiVel(:,3))/dt, 150);


figure(10);
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

% instead of accelerometer, use body angles
wRb_ax = g* -sin(angBody(:,2)./(cos(angBody(:,2)).*cos(angBody(:,1))));
subplot(5,3,13);
plot(t, wRb_ax);
title('droneAcc');

subplot(5,3,14);
wRb_ay = g* sin(angBody(:,1)) .* sin(angBody(:,2));
plot(t, wRb_ay);
title('droneAcc');

subplot(5,3,15);
wRb_az = g * cos(angBody(:,1)) .* cos(angBody(:,2));
plot(t, wRb_az);
title('droneAcc');
%%
ps_ax = wRb_ax - optiAcc(:,1);
% plot(t, ps_ax, '.');
% title('ot minus estimated acc');
% 
% subplot(2,1,2);
% plot(t(2880:3480), ps_ax(2880:3480));
% title('some samples only');

figure(13);
subplot(2,1,1);
drag_k = (wRb_ax - optiAcc(:,1))./optiVel(:,1); 
drag_k(find(drag_k > 27)) = 0;
drag_k(find(drag_k < -27)) = 0;
plot(t, drag_k, '.');
title('coeff with time'); xlabel('time'); ylabel('k_D');

lim = 1:size(t,1);

subplot(2,1,2);
plot(t(lim), drag_k(lim), '.');
title('some samples only'); xlabel('time'); ylabel('k_D');



figure(14);
subplot(2,2,1);
plot(t(lim), optiVel(lim,1));
title('v_x with time'); xlabel('time'); ylabel('v_x');

subplot(2,2,2);
plot(t(lim), drag_k(lim,1));
title('k_D with time'); xlabel('time'); ylabel('k_D');

subplot(2,2,3);
plot(optiVel(lim,1), drag_k(lim), '.');
xlabel('v_x'); ylabel('k_D'); 

subplot(2,2,4);
plot(optiVel(lim,1), optiAcc(lim, 1), '.');
xlabel('v_x'); ylabel('a_x');

% forward pitching manuever
% 2880 3480
% 2680 3704

%% todo:
% forward model, compare plots GT vs estimated on cyberzoo
% already have GT plots, can I do everything with it? Yes, maybe the model
% doesn't require much. It estimates the pos and vel in inertial just by
% the model. 
% TODO: Import paparazzi model. Unsure if phi theta psi are in body or
% world, after that I plot with real GT position readings!! 
% 
%   float az = DR_FILTER_GRAVITY / cosf(theta * DR_FILTER_THRUSTCORR) / cosf(phi * DR_FILTER_THRUSTCORR);
%   float abx =  sinf(-theta) * az;
%   float aby =  sinf(phi)   * az;
% 
%   // Earth accelerations
%   float ax =  cosf(psi) * abx - sinf(psi) * aby - dr_state.vx * DR_FILTER_DRAG ;
%   float ay =  sinf(psi) * abx + cosf(psi) * aby - dr_state.vy * DR_FILTER_DRAG;
% 
% 
%   // Velocity and Position
%   dr_state.vx += ax * dt;
%   dr_state.vy += ay * dt;
%   dr_state.x += dr_state.vx * dt;
%   dr_state.y += dr_state.vy * dt;
vx = zeros(size(t, 1), 1);
vy = zeros(size(t, 1), 1);
x  = zeros(size(t, 1), 1);
y  = zeros(size(t, 1), 1);

for i = 2:1:size(t,1)
    
    az  = g ./ (angBody(i,1) .* angBody(i,2));

    abx = sin(-angBody(i,2)) * az;
    aby = sin(angBody(i,1))  * az;

    ax = cos(angBody(i,3)) * abx - sin(angBody(i,3)) * aby;
    ay = sin(angBody(i,3)) * abx + cos(angBody(i,3)) * aby;

    vx(i) = vx(i-1) + ax*dt;
    vy(i) = vy(i-1) + ay*dt;
    
    x(i) = x(i-1) + vx(i)*dt;
    y(i) = y(i-1) + vy(i)*dt;
end

%% plot world and body angles - todo:!!!
%     R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
%         sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
%         sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
%         cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
%         cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];