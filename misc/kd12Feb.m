%% Below code is useless
% psi theta phi
world2body = zeros(3,3,size(M,1));
world2body = eul2rotm([angBody(:,1), angBody(:,2), angBody(:,3)]);


psuedo_acc = zeros(3,1,size(M,1));

for i = 1:1:size(M,1)
    body2world = world2body(:,:,i)';
    psuedo_acc(:,:,i) = body2world * ([ot.acc.x(i); ot.acc.y(i); ot.acc.z(i)] - [0; 0; gravity]);
end

[roll(:,1), pitch(:,1), yaw(:,1)] = body2world 

figure(13);

plot(t, angBody(:,1));


%% calc velocity and acc

dt = 1/512;
velocity.x = gradient(M(:,14))/dt;
velocity.y = gradient(M(:,15))/dt;
velocity.z = gradient(M(:,16))/dt;

velocity.x = lowpass(velocity.x, 2, 1/dt);
velocity.y = lowpass(velocity.y, 2, 1/dt);
velocity.z = lowpass(velocity.z, 2, 1/dt);

acc.x = gradient(velocity.x)/dt;
acc.y = gradient(velocity.y)/dt;
acc.z = gradient(velocity.z)/dt;

acc.x = lowpass(acc.x, 2, 1/dt);
acc.y = lowpass(acc.y, 2, 1/dt);
acc.z = lowpass(acc.z, 2, 1/dt);

theta = M(:,12);

k_arr = (acc.x - (9.81*(theta)))./velocity.x.^2;

k_arr(isnan(k_arr)) = 0;
k_arr(isinf(k_arr)) = 0;



%%
% or use diff
dt = 0.002;
gravity = 9.812;

ot.vel.x = lowpass(gradient(M(:,22))/dt, 10, 1/dt);
ot.vel.y = lowpass(gradient(M(:,23))/dt, 10, 1/dt);
ot.vel.z = lowpass(gradient(M(:,24))/dt, 10, 1/dt);

ot.acc.x = lowpass(gradient(ot.vel.x)/dt, 10, 1/dt);
ot.acc.y = lowpass(gradient(ot.vel.y)/dt, 10, 1/dt);
ot.acc.z = lowpass(gradient(ot.vel.z)/dt, 10, 1/dt);

% ypr
% psi theta phi
wRb = zeros(3,3,size(M,1));
angle1 = lowpass(M(:,21), 10, 1/dt);
angle2 = lowpass(M(:,21), 10, 1/dt);
angle3 = lowpass(M(:,21), 10, 1/dt);
wRb = eul2rotm([angle1, angle2, angle3]);

psuedo_acc = zeros(3,1,size(M,1));

for i = 1:1:size(M,1)
    psuedo_acc(:,:,i) = wRb(:,:,i)'* ([ot.acc.x(i); ot.acc.y(i); ot.acc.z(i)] - [0; 0; gravity]);
end

figure;
subplot(3,1,1);
plot(M(:,5)/1024);
title('as seen by IMU acc');
legend('acclerometer');
subplot(3,1,2);
plot(psuedo_acc(1,:)');
title('as seen by OT in body frame');
legend('ot');
subplot(3,1,3);
magic_diff = M(:,5)/1024 - psuedo_acc(1,:)';
plot(magic_diff);
title('psuedoness - difference between what acc says and what OT says');
legend('diff');

figure;
subplot(3,1,1);
plot(M(:,6)/1024);
title('as seen by IMU acc');
legend('acclerometer');
subplot(3,1,2);
plot(psuedo_acc(2,:)');
title('as seen by OT in body frame');
legend('ot');
subplot(3,1,3);
magic_diff = M(:,6)/1024 - psuedo_acc(2,:)';
plot(magic_diff);
title('psuedoness - difference between what acc says and what OT says');
legend('diff');

%%
figure;
subplot(3,1,1);
plot(M(:,7)/1024);
title('as seen by IMU acc');
legend('acclerometer');
subplot(3,1,2);
plot(psuedo_acc(3,:)');
title('as seen by OT in body frame');
legend('ot');
subplot(3,1,3);
magic_diff = M(:,7)/1024 - psuedo_acc(3,:)';
plot(magic_diff);
title('psuedoness - difference between what acc says and what OT says');
legend('diff');


filt_acc_z = lowpass(M(:,7)/1024, 10, 1/dt);
figure;
plot(filt_acc_z);