% input args
% angBody: ypr in world frame for rotating acceleration from opti
% filt_a: filtered accelerometer for plot compare
% time: for plot

% output args
% T: thrust (acc_z) in body frame 
function T = thrustMatch(angBody, optiAcc, filt_a, t)

T = zeros(length(t),1);
for i = 1:1:length(t)
    
    phi = angBody(i,1);
    theta = angBody(i,2);
    psi = angBody(i,3);
      
    R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    
    thrust_temp = R * [optiAcc(i,1); optiAcc(i,2); (-9.81 + optiAcc(i,3))]; % laterals shouldn't be zero
    T(i,1) = thrust_temp(3); % time to match with body z (accelerometer)
    
end

% compare thrust with acc_zB
figure; 
plot(t, filt_a(:,3)); hold on; grid on;
plot(t, T(:,1));
plot(t, -9.81 * ones(length(t),1));
legend('filtered_{az}', 'rotated optiAcc to thrust');
xlabel('time');
ylabel('a_z (m/s^2)');
title('compare thrust');