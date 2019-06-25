%% TO find the psuedoaccelerations on body fixed frame
% pending: yaw in the rotation matrix is it correct

close all;
filename = '2019-02-19_13_57_58.csv';
% ["counter","accel_unscaled_x","accel_unscaled_y","accel_unscaled_z","gyro_unscaled_p","gyro_unscaled_q","gyro_unscaled_r","mag_unscaled_x","mag_unscaled_y","mag_unscaled_z","phi","theta","psi","opti_x","opti_y","opti_z","time"]
M = csvread(filename, 1, 0);
col = size(M,2);

fid = fopen(filename);
a = textscan(fid,'%s',1);
C = strsplit(string(a),',');
fclose(fid);

accel = M(:,2:4)./512;
gyro  = M(:,5:7)./512;
mag   = M(:,8:10);
angBody = M(:,11:13);
optiPos = M(:,14:16);
t = M(:,end);

for i=1:1:col
     % figure('Name', C(i));
     % plot(M(:,i));
end

%% plot opti x, xd, xdd
% todo: verify imu conventions
dt = 1/512;
g =  9.81;

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
wRb_ay = g * sin(angBody(:,1)) .* sin(angBody(:,2));
plot(t, wRb_ay);
title('droneAcc');

subplot(5,3,15);
wRb_az = g * cos(angBody(:,1)) .* cos(angBody(:,2));
plot(t, wRb_az);
title('droneAcc');


%%
figure(12);
ps_ax = wRb_ax - optiAcc(:,1);
%plot(t, ps_ax, '.'); hold on; 
plot(t, optiAcc(:,1)); hold on; plot(t, wRb_ax);
grid on;
x = linspace(t(1), t(end), length(t));
p = polyfit(x', ps_ax, 1);
y1 = polyval(p, x);
% plot(x, y1, 'x');

legend('opti', 'drone'); %, 'polyfit');
% title('ot minus estimated acc');
% 
% subplot(2,1,2);
% plot(t(2880:3480), ps_ax(2880:3480));
% title('some samples only');

figure(13);
drag_k = (wRb_ax - optiAcc(:,1))./optiVel(:,1); 
drag_k(find(drag_k > 27)) = 0;
drag_k(find(drag_k < -27)) = 0;
plot(t, drag_k, '.');
title('coeff with time'); xlabel('time'); ylabel('k_D');

%% 
lim = 1:size(t,1);
figure(14);
plot(t(lim), drag_k(lim), '.');
title('some samples only'); xlabel('time'); ylabel('k_D'); grid on;


figure(15);
subplot(2,2,1);
plot(t(lim), optiVel(lim,1));
title('v_x with time'); xlabel('time'); ylabel('v_x'); grid on;

subplot(2,2,2);
plot(t(lim), drag_k(lim,1));
title('k_D with time'); xlabel('time'); ylabel('k_D'); grid on;

subplot(2,2,3);
plot(optiVel(lim,1), drag_k(lim), '.'); grid on;
xlabel('v_x'); ylabel('k_D'); 

subplot(2,2,4);
% plot(optiVel(lim,1), optiAcc(lim, 1), '.'); grid on;
 plot(optiVel(lim,1), wRb_ax, '.'); grid on;
xlabel('v_x'); ylabel('a_x');

% forward pitching manuever
% 2880 3480
% 2680 3704



%% 

yaxisData = cos(angBody(:,2)) .* cos(angBody(:,3)) .* optiAcc(:,1) ...
          + cos(angBody(:,2)) .* sin(angBody(:,3)) .* optiAcc(:,2) ...
          - sin(angBody(:,2)) .* optiAcc(:,3);
      
xaxisData = cos(angBody(:,2)) .* optiVel(:,1);
%        ...
%           + cos(angBody(:,2)) .* sin(angBody(:,3)) .* optiVel(:,2) ...
%           - sin(angBody(:,2)) .* optiVel(:,3);
      
plot(xaxisData(:,1), yaxisData(:,1), '.');
          
          