clear all; clc; close all;
%%
plant = tf(1,[1 0 0]);
Ts = 0.1;
p = 10; % prediction horizon
m = 3;  % control horizon
mpcobj = mpc(plant, Ts, p, m);

mpcobj.MV = struct('Min', -1, 'Max', 1);

mdl = 'mpc_doubleint';
open_system(mdl);
sim(mdl);


%% try to propogate other dynamics


A = [1 0.025 0.0031 0;
    0  1 0.2453 0;
    0 0 0.7969 0.0225;
    0 0 -1.7976 0.9767];

B = [0;0;0.01; 0.9921];

t = 0:0.025:5;
state = zeros(length(t), 4);
u = 0;
in = zeros(length(t), 1);

for i=2:1:length(t)
    
    if t(i) > 0.5
        u = 20 * 3.142/180;
    end
    in(i) = u;
    state(i, :) = (A * state(i-1, :)' + B * tan(u))';
end
%% 
figure; 
subplot(2,1,1);
plot(t, state(:,1)); hold on;
plot(t, state(:,2));
legend('x', 'dot{x}');
subplot(2,1,2);
plot(t, state(:,3)* 180 / 3.142); hold on;
plot(t, state(:,4)* 180 / 3.142);
plot(t, in * 180 / 3.142); 
legend('theta', 'dot{theta}', 'input');

%% 
syms g kd h sig;
A = [-kd 0; 1 0];
B = [g; 0];

%% 

H = tf(9.81, [1 0.5 0]);
Hd = c2d(H, 0.01, 'zoh');
step(H,'-',Hd,'--');


%% try quadprog
% continuous linear
A = [-0.5 0; 1 0]; 
B = [9.81; 0];
C = [0, 1];
D = [0];

sys = ss(A, B, C, D);
h = 0.01;
sysZ = c2d(sys, h, 'zoh');


N = 35;  % give only 1second
P = [1 0; 0 100]; % position reprimanded more
R  = zeros(2, N);
tR = zeros(2, N);
x0 = [1.0; -0.5];

% for i = 1:1:N
%     tR(:,i) = sysZ.A^(i-1) * sysZ.B;
%     R(:, N-i+1) = tR(:,i);
% end

for i = 0:1:N-1
    R(:,i+1) = (sysZ.A)^(N-i-1) * (sysZ.B);
end
R 
H = 2 * (R' * P * R);
f = 2 * ((sysZ.A)^N * x0)' * P * R;

% one radian is 57 degrees max bank angle
ub =  1 * ones(N,1);
lb = -1 * ones(N,1);
% ub = []; lb = [];
options = optimoptions('quadprog','Display','iter', 'Algorithm', 'trust-region-reflective');
[U,fval,exitflag,output,lambda] = quadprog(H,f,[],[],[],[], lb, ub, x0, options)

% emulate 
states = zeros(2,N);
states(:,1) = x0;
for i = 2:1:N
   states(:,i) = (sysZ.A) * states(:,i-1) + (sysZ.B) * U(i-1);
end

t = (0:1:N-1)*h;
plot(t, states(:,:));
grid on;
xlabel('time'); ylabel('states');
legend('vel', 'pos');


%% lsim try for continuous nonlinear vs continuous linear

% continuous linear
A = [-0.5 1; 1 0]; 
B = [9.81; 0];
C = [0, 1];
D = [0];

sys = ss(A, B, C, D);
h = 0.01;

t = 0:h:2;
u = (20 * 3.142 / 180) * ones(length(t), 1);
x0 = [0, -0.5];
y = lsim(sys, u, t, x0);
plot(t, y); hold on; grid on;

% now continuous nonlinear
dt = mean(gradient(t));
% maybe dt = h

flap = 1;
phi = u(1);
i = 1;
x_r = length(t);
v = x0(1);
x = x0(2);

for t=0:h:2
    % T =  9.81 / (cos(phi * flap));
    % ay = sin(phi) * T - v * 0.5;
    ay = 9.81 * tan(phi) - v * 0.5;

    % Simulation
    % ct = ct + dt;
    v = v + ay * dt;
    x = x + v  * dt;
    x_r(i) = x;
    i = i+1;
end

t=0:h:2;
plot(t, x_r);

% discretized linear

sysZ = c2d(sys, h, 'zoh');
yd = lsim(sysZ, u, t, x0);
plot(t, yd);
