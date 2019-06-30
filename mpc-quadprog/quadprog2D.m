%% try quadprog

% AIM:
% move from state [1, -0.5] to [0, 0] 
% terminal cost  is the quadprog Cost function

% TO FIND: 
% Find a sequence of u, which can help us reach the state [0, 0]

% GIVEN:
% continuous time dynamics
% initial and final states
% weights in the cost function
% horizon and sampling time is not limited - can be arbitrary
% constraints on maximum bounds of inputs to be given to the system 

% PROCEDURE:
% expand the terminal cost function, write it in terms of U
% optimize for U subject to constraints using quadprop -
% interior-point-convex

pos0 = [-24, -7];
posf = [-0.37, -12.23];

vel0 = [0, 0];
velf = [6, 0];

x0 = [vel0(1); pos0(1); vel0(2); pos0(2)];
xd = [velf(1); posf(1); velf(2); posf(2)];

h = 0.1;
T = (sqrt((pos0(1) - posf(1))^2 + (pos0(2) - posf(2))^2)) / 4;
% N = 600;           % 2 seconds
N = round(T/h)

%% STEP1: Populate matrices

% continuous linear
A = [-0.5, 0, 0, 0; 
       1, 0,  0, 0;
       0, 0, -0.5, 0;
       0, 0,  1,  0];
  
B = [9.81, 0; 
      0, 0; 
      0, 9.81; 
      0, 0];
  
C = [0, 1, 0, 1];
D = [0, 0];

sys = ss(A, B, C, D);

sysZ = c2d(sys, h, 'zoh');

%%



P = [1, 0, 0, 0; 
    0, 10, 0, 0; 
    0, 0,  1, 0;
    0, 0,  0, 10]; % position reprimanded more
R  = zeros(4, 2 * N);

for i = 0:1:N-1
    col_idx = (1 + 2*i : 2 + 2*i);
    R(:,col_idx) = (sysZ.A)^(N-i-1) * (sysZ.B);
end
H = 2 * (R' * P * R);
H = 0.5 * (H + H' + 0.5 * eye(size(H)));
% check if positive definite for convergence?
min(eig(H))
% H = 0.5 * (H + H' + eye(size(H)));


f = 2 * (((sysZ.A)^N * x0)' - xd') * P * R;

maxbank = 35 * 3.142 / 180;

ub =  maxbank * ones(2 * N, 1);  % one radian is 57 degrees max bank angle
lb = -maxbank * ones(2 * N, 1);  % one radian is 57 degrees max bank angle

%% STEP2: Optimize for U - PROBLEM! fval doesn't decrease monotonically!!

% optimize
options = optimoptions('quadprog','Display','iter'); %, 'Algorithm', 'trust-region-reflective');
[U,fval,exitflag,output,lambda] = quadprog(H,f,[],[],[],[], lb, ub, x0, options)


% check the % banged
t = (0:1:N-1) * h;
theta_arr = zeros(N,1);
phi_arr   = zeros(N,1);
for i = 1:1:N
    theta_arr(i) = U(2*i - 1);
    phi_arr(i)   = -1 * U(2*i);
    
end


%% 
figure;
plot(t, rad2deg(theta_arr)); hold on;
plot(t, rad2deg(phi_arr)); grid on;
legend('\theta', '\phi');
xlabel('time'); ylabel('bank angles');
banging_theta = sum(abs(theta_arr))/(N * maxbank) * 100
banging_phi = sum(abs(phi_arr))/(N * maxbank) * 100


%% STEP3: emulate the optimization on system model

states = zeros(4, N);
states(:,1) = x0;

vel = zeros(N, 2);
pos = zeros(N, 2);

vel(1,:) = vel0;
pos(1,:) = pos0;

dt = h;
psi = 10 * 3.142 / 180;

flap = 0.8;
% flap = 1;
for i = 2:1:N
    
    theta = U(2*i -3);
    phi   = -1 * U(2*i -2);
    
    states(:,i) = (sysZ.A) * states(:,i-1) + (sysZ.B) * [theta; -phi];
    
    newAng = [cos(psi) -sin(psi); sin(psi) cos(psi)] * [theta; phi];
    theta = newAng(1);
    phi = newAng(2);
    
    T =  9.81 / (cos(phi * flap) * cos(theta * flap));
    ax = (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * T - vel(i-1, 1) * 0.5;
    ay = (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * T - vel(i-1, 2) * 0.5;

    % Simulation
    vel(i,:) = vel(i-1,:) + [ax ay] .* dt;
    pos(i,:) = pos(i-1,:) + vel(i,:) .* dt;
end

%% STEP4: plot the emulation
figure;
plot(t, states(:,:)); hold on;
text(t(1), states(1,1), '\dot{x} start');
text(t(1), states(2,1), 'x start');
text(t(1), states(3,1), '\dot{y} start');
text(t(1), states(4,1), 'y start');

plot(t, pos);
plot(t, vel);

text(t(end),posf(1,1),'desired x pos');
text(t(end),posf(1,2),'desired y pos');

text(t(end),velf(1,1),'desired x vel');
text(t(end),velf(1,2),'desired y vel');

grid on;
xlabel('time'); ylabel('states');
legend('velx', 'posx', 'vely', 'posy', 'posx nonlinear', 'posy nonlinear', 'velx nonlinear', 'vely nonlinear');

%%

figure;
plot(pos(:,1), pos(:,2)); hold on;
text(pos(1,1), pos(1,2), 'start');
plot(posf(1,1), posf(1,2), 'xr');
plot(states(2,:), states(4,:));
grid on; 
xlabel('x(m)'); ylabel('y(m)');
legend('nonlinear','linear prog cost');