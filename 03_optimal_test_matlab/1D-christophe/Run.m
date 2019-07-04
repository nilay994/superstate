clear all;
clc;
close all;


dt = 1/100;
t = 0:dt:10;

len = size(t,2);

% simulate
phi = zeros(len,1);
theta = zeros(len,1);
x = zeros(len,2);
v = zeros(len,2);


% target
% test3
% xd = [-0.37, -12.23];
% vd = [5, 0];
% cyberzoo
xd = [0, 0];
vd = [2, 0];

% initialize
% test3
% x(1,:) = [-24, 5];
% v(1,:) = [2, 0];
% cyberzoo
x(1,:) = [-4.5, -2.5];
v(1,:) = [0, 0];

% Gains
K_ff_theta = 14/57 / 5;   % rad to fly at (e.g. 10 deg = 5 m/s)
K_p_v = 1;
K_p_theta = 6 / 57;       % m/s to radians
maxbank = 25 / 57;

x0 = x(1,:);
v0 = v(1,:);


[phi0, phi1, t1] = optimizer(x0, v0, xd, vd)

p_max = 400 / 57 * dt;
optimal = 1;
flap = 0.85;

for i=2:len
    psi = 0;
    % Simulate until passing the gate
    if (x(i-1,1) > 2)
        break;
    end
    
    % Control
    theta(i) = (vd(1) - v(i-1,1)) * K_p_theta + vd(1) .* K_ff_theta;  %todo: unsure about feedfwd term
    if (theta(i) > maxbank)
        theta(i) = maxbank;
    elseif (theta(i) < -maxbank)
        theta(i) = -maxbank;
    end
    
    d_theta = theta(i) - theta(i-1);
    if (d_theta > p_max)
        d_theta = p_max;
    elseif (d_theta < -p_max)
        d_theta = -p_max;
    end
    
    theta(i) = theta(i-1) + d_theta;
    
    % ax = sin(theta(i)) * 9.81  / cos(phi(i)) - v(i-1, 1) * 0.55;
    
    % Lateral PID
    x_err = xd(2) - x(i-1,2);
    v_cmd = x_err * K_p_v;
    phi_cmd = (v_cmd - v(i-1,2)) * K_p_theta + v_cmd  .* K_ff_theta;

    
    % Optimal CTRL
    if (optimal)
        if t(i) < t1
            phi_cmd = phi0;
        else
            phi_cmd = phi1;
        end
    end
    
    % until after passing the gate
    if(abs(x(i-1, 1) - xd(1)) < 0.1)
        phi_cmd = 0;
    end
    
    % roll reference model
    d_phi = phi_cmd - phi(i-1);
    if (d_phi > p_max)
        d_phi = p_max;
    elseif (d_phi < -p_max)
        d_phi = -p_max;
    end
    phi(i) = phi(i-1) + d_phi;
    % ay = - sin(phi(i)) * 9.81  / cos(phi(i) * 0.8) - v(i-1, 2) * 0.55;
    
    T =  9.81 / (cos(phi(i) * flap) * cos(theta(i) * flap));
    ax = (cos(phi(i)) * sin(theta(i)) * cos(psi) + sin(phi(i)) * sin(psi)) * T - v(i-1, 1) * 0.5;
    ay = (cos(phi(i)) * sin(theta(i)) * sin(psi) - sin(phi(i)) * cos(psi)) * T - v(i-1, 2) * 0.5;
    
    % Simulation
    dv = [ax ay];
    v(i,:) = v(i-1,:) + dv .* dt;
    x(i,:) = x(i-1,:) + v(i,:) .* dt;
    
end


%% plot

figure(1)
range = 1:i-1;
subplot(3,1,1);
hold off
plot(t(range),phi(range) .* 180 ./ pi)
hold on
plot(t(range),theta(range) .* 180 ./ pi)
grid on
ylabel('\phi, \theta [deg]')
legend('\phi', '\theta')

subplot(3,1,2);
hold off
plot(t(range),v(range,:))
grid on
ylabel('V [m/s]')
xlabel('Time [t]')
legend('v_x', 'v_y')

subplot(3,1,3);
hold off
plot(x(range,1),x(range,2))
hold on;
plot(xd(1), xd(2),'xr');
grid on
ylabel('Y [m]')
xlabel('X [m]')