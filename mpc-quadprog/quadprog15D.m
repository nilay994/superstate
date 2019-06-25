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

pos0 = [-5, -1];
posf = [0, 1];

vel0 = [2, 0];
velf = [2, 0];


%% STEP1: Populate matrices

% continuous linear
A = [-0.5 0; 1 0]; 
B = [9.81; 0];
C = [0, 1];
D = [0];

x0 = [vel0(2); pos0(2)];
xd = [velf(2); posf(2)];

sys = ss(A, B, C, D);
h = 0.01;
sysZ = c2d(sys, h, 'zoh');

N = 200;           % 2 seconds
P = [1 0; 0 100]; % position reprimanded more
R  = zeros(2, N);

for i = 0:1:N-1
    R(:,i+1) = (sysZ.A)^(N-i-1) * (sysZ.B);
end
H = 2 * (R' * P * R);

% check if positive definite for convergence?
eig(H)

f = 2 * (((sysZ.A)^N * x0)' - xd') * P * R;

maxbank = 45/57;

ub =  maxbank * ones(N,1);  % one radian is 57 degrees max bank angle
lb = -maxbank * ones(N,1);  % one radian is 57 degrees max bank angle

%% STEP2: Optimize for U - PROBLEM! fval doesn't decrease monotonically!!

% optimize
options = optimoptions('quadprog','Display','iter'); %, 'Algorithm', 'trust-region-reflective');
[U,fval,exitflag,output,lambda] = quadprog(H,f,[],[],[],[], lb, ub, x0, options)

%% STEP3: emulate the optimization on system model

states = zeros(2,N);
states(:,1) = x0;

vel = zeros(N,2);
pos = zeros(N,2);

vel(1,:) = vel0;
pos(1,:) = pos0;

dt = h;
psi = 0;

K_ff_theta = 20/57 / 5;   % rad to fly at (e.g. 10 deg = 5 m/s)
K_p_theta = 10 / 57;       % m/s to radians
flap = 1;

for i = 2:1:N
    states(:,i) = (sysZ.A) * states(:,i-1) + (sysZ.B) * U(i-1);

    phi = -U(i-1);

    % Control
    theta = (velf(1) - vel(i-1,1)) * K_p_theta + velf(1) .* K_ff_theta;  %todo: unsure about feedfwd term
    if (theta > maxbank)
        theta = maxbank;
    elseif (theta < -maxbank)
        theta = -maxbank;
    end

    T =  9.81 / (cos(phi * flap) * cos(theta * flap));
    ax = (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * T - vel(i-1, 1) * 0.5;
    ay = (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * T - vel(i-1, 2) * 0.5;

    % Simulation
    vel(i,:) = vel(i-1,:) + [ax ay] .* dt;
    pos(i,:) = pos(i-1,:) + vel(i,:) .* dt;
end

%% STEP4: plot the emulation

t = (0:1:N-1) * h;
plot(t, states(:,:)); hold on;
text(t(1), states(1,1), 'vel start');
text(t(1), states(2,1), 'pos start');

plot(t, pos);
plot(t, vel);

plot(t(end),xd(1),'xr');
plot(t(end),xd(2),'xr');

grid on;
xlabel('time'); ylabel('states');
legend('vel', 'pos', 'nonlinear model vel', 'nonlinear model pos');

%%

figure;
plot(pos(:,1), pos(:,2)); hold on;
plot(posf(1,1), posf(1,2), 'xr');
