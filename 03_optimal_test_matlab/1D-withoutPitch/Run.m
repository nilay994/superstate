close all; clc; clear all;
dt = 1/500;
t = 0:dt:5;

len = size(t,2);

% simulate
phi = zeros(len,1);
pos = zeros(len,1);
vel = zeros(len,1);


% Target
yd = 0;
vyd = 0;


% Initialize
pos(1,1) = -20;
vel(1,1) = 0;


% Gains
K_p_theta = 6 / 57;       % m/s -> radians
K_ff_theta = 14/57 / 5;   % rad to fly at (e.g. 10 deg = 5 m/s)
K_p_v = 1;
K_p_theta = 6 / 57;
maxbank = 45 / 57;


[phi0, phi1, t1] = optimizer(pos(1,1), 0, t(end), 1)


%% go now!

% maximum roll rate
pmax = 400 / 57 * dt;
optimal = 1

% Target
yd = 0;
vy_d = 0;

for i=2:len
    % Simulate until passing the gate
    if (pos(i-1,1) > 0)
        break;
    end
    
%     % Control
%     theta(i) = (tv - vel(i-1)) * K_p_theta + tv .* K_ff_theta;
%     if (theta(i) > maxbank)
%         theta(i) = maxbank;
%     elseif (theta(i) < -maxbank)
%         theta(i) = -maxbank;
%     end
%     ax = sin(theta(i)) * 9.81  / cos(phi(i)) - vel(i-1, 1) * 0.55;
    
    % Lateral PID
    y_err = yd - pos(i-1,1);
    v_cmd = y_err * K_p_v;
    phi_cmd = (v_cmd - vel(i-1,1)) * K_p_theta + v_cmd  .* K_ff_theta;

    
    % Optimal CTRL
    if (optimal)
        if t(i) < t1
            phi_cmd = phi0;
        else
            phi_cmd = phi1;
        end
    end
    
    if (pos(i-1,1) > 0)
        phi_cmd = 0;
    end
    
    % roll reference model
    d_phi = phi_cmd - phi(i-1);
    if (d_phi > pmax)
        d_phi = pmax;
    elseif (d_phi < -pmax)
        d_phi = -pmax;
    end
    
    phi(i) = phi(i-1) + d_phi;
    ay = sin(phi(i)) * 9.81  / cos(phi(i) * 0.8) - vel(i-1,1) * 0.51;
    
    % Simulation
    vel(i,1) = vel(i-1,1) + ay .* dt;
    pos(i,1) = pos(i-1,1) + vel(i,1) * dt;
    
end


%% plot

figure(1)
range = 1:i-1;
subplot(3,1,1)
hold off
plot(t(range),phi(range) .* 180 ./ pi)
hold on
%plot(t (range),theta(range) .* 180 ./ pi)
grid on
ylabel('\phi, \theta [deg]')
legend('\phi', '\theta')

subplot(3,1,2)
hold off
plot(t(range),vel(range,:))
grid on
ylabel('V [m/s]')
xlabel('Time [t]')
legend('v_x', 'V_y')

% subplot(3,1,3)
% hold off
% plot(pos(range,1),pos(range,2))
% hold on
% plot([0 0.25 0.25 0 0],[0.5 0.5 -0.5 -0.5 0.5],'r')
% grid on
% ylabel('Y [m]')
% xlabel('X [m]')

subplot(3,1,3)
hold off
plot(t(range), pos(range,:))
hold on
plot([0 0.25 0.25 0 0],[0.5 0.5 -0.5 -0.5 0.5],'r')
grid on
ylabel('Y [m]')
xlabel('time [sec]')