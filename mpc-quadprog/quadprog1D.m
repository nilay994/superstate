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

%% STEP1: Populate matrices

% continuous linear
A = [-0.5 0; 1 0]; 
B = [9.81; 0];
C = [0, 1];
D = [0];

x0 = [2.0; -1];
xd = [2.0; 1];

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

ub =  45/57 * ones(N,1);  % one radian is 57 degrees max bank angle
lb = -45/57 * ones(N,1);  % one radian is 57 degrees max bank angle

%% STEP2: Optimize for U - PROBLEM! fval doesn't decrease monotonically!!

% optimize
options = optimoptions('quadprog','Display','iter'); %, 'Algorithm', 'trust-region-reflective');
[U,fval,exitflag,output,lambda] = quadprog(H,f,[],[],[],[], lb, ub, x0, options)

%% STEP3: emulate the optimization on system model

states = zeros(2,N);
states(:,1) = x0;

vy   = zeros(N,1);
posy = zeros(N,1);

vy(1)   = x0(1);
posy(1) = x0(2);
dt = h;

for i = 2:1:N
   states(:,i) = (sysZ.A) * states(:,i-1) + (sysZ.B) * U(i-1);
   phi = U(i-1);
   T  = 9.81/cos(phi);
   ay = tan(phi) * 9.81 - 0.5 * vy(i-1); 
   vy(i,1) = vy(i-1,1) + ay * dt;
   posy(i,1) = posy(i-1,1) + vy(i-1,1) * dt + 0.5 * ay * dt^2;
end

%% STEP4: plot the emulation

t = (0:1:N-1) * h;
plot(t, states(:,:)); hold on;
text(t(1), states(1,1), 'vel start');
text(t(1), states(2,1), 'pos start');

plot(t, posy);
plot(t, vy);

plot(t(end),xd(1),'xr');
plot(t(end),xd(2),'xr');

grid on;
xlabel('time'); ylabel('states');
legend('vel', 'pos', 'nonlinear model vel', 'nonlinear model pos');

