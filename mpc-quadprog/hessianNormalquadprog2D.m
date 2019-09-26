%% try quadprog
% PROBLEM - this method is self - violating
% you are pushing for saturation using linear state space models
% this already breaks down at 40 degrees.
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

% clc; close all; clear all;

% STEP0: setup for time optimal secant method
tries = 10; 

banging_theta = zeros(tries,1);
banging_phi   = zeros(tries,1);
blah = 2;
%for blah = 1:1:tries
while ((banging_theta(blah-1)) < 35 && (banging_phi(blah-1) < 35))
%     pos0 = [-2.7, 2.15];
%     posf = [2.2, -2.5]; 
% 
%     vel0 = [0, 0];
%     velf = [0, -2];

%     %pos0 = [2.1998,  9.0017];
%     pos0 = [50, 20];
%     posf = [-7.3087, -12.1368]; 
% 
%     vel0 = [0, -4];
%     velf = [0, -4];

    pos0 = [-20 -10];
    posf = [0 0];
    
    vel0 = [-3, 0];
    velf = [-3, 0];

    x0 = [vel0(1); pos0(1); vel0(2); pos0(2)];
    xd = [velf(1); posf(1); velf(2); posf(2)];

    h = 0.1;
    totalT = 10 * (sqrt((pos0(1) - posf(1))^2 + (pos0(2) - posf(2))^2)) / ...
             ((sqrt((vel0(1) - velf(1))^2 + (vel0(2) - velf(2))^2)) * blah) ;
         
    totalT = 2 * (sqrt((pos0(1) - posf(1))^2 + (pos0(2) - posf(2))^2)) / ...
             ( blah) ;
    % N = 600;           % 2 seconds
    N = round(totalT/h);

    % STEP1: Populate matrices

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

    % STEP2: get ready for quadprog

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
    
    f = 2 * (((sysZ.A)^N * x0)' - xd') * P * R;

    maxbank = 25 * 3.142 / 180;

    ub =  maxbank * ones(2 * N, 1);  % one radian is 57 degrees max bank angle
    lb = -maxbank * ones(2 * N, 1);  % one radian is 57 degrees max bank angle

    % STEP3: Optimize for U - PROBLEM! fval doesn't decrease monotonically!!

    % optimize
    options = optimoptions('quadprog','Display','iter'); %, 'Algorithm', 'trust-region-reflective');
    [U, fval, exitflag, output, lambda] = quadprog(H,f,[],[],[],[], lb, ub, x0, options);
    
    bangedtheta_acc = 0;
    bangedphi_acc = 0;

    % check the % banged
    t = (0:1:N-1) * h;
    theta_arr = zeros(N,1);
    phi_arr   = zeros(N,1);
    for i = 1:1:N
        theta_arr(i) = U(2*i - 1);
        phi_arr(i)   = -1 * U(2*i);
        % if ((abs(abs(theta_arr(i) - maxbank))<0.01) || (abs(abs(phi_arr(i))-maxbank) < 0.01))
        if ((abs(abs(theta_arr(i) - maxbank))<0.01) || (abs(abs(phi_arr(i) - maxbank))<0.01))
            bangedtheta_acc = bangedtheta_acc + abs(theta_arr(i));
            bangedphi_acc   = bangedphi_acc + abs(phi_arr(i));
        end
    end

    banging_theta(blah) = bangedtheta_acc / (N * maxbank) * 100;
    banging_phi(blah)   = bangedphi_acc   / (N * maxbank) * 100;
    
    
    % PLOT THE % banged
%     figure;
%     plot(t, rad2deg(theta_arr)); hold on;
%     plot(t, rad2deg(phi_arr)); grid on;
%     hold off;
%     legend('\theta', '\phi');
    % xlabel('time'); ylabel('bank angles');
%     banging_theta(blah) = sum(abs(theta_arr))/(N * maxbank) * 100
%     banging_phi(blah) = sum(abs(phi_arr))/(N * maxbank) * 100
    % NOTE: lambda already gives you a measure of how close you are to constraints
    
    % STEP4: emulate the optimization on system model
    states = zeros(4, N);
    states(:,1) = x0;

    vel = zeros(N, 2);
    pos = zeros(N, 2);

    vel(1,:) = vel0;
    pos(1,:) = pos0;

    dt = h;
    psi = 0; %10 * 3.142 / 180;

    flap = 0.8;
    % flap = 1;
    absvel = zeros(N,1);
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
        
        absvel(i,1) = sqrt(vel(i,1)^2 + vel(i,2)^2);
    end

    costst = (states(:,N) - [velf(1); posf(1); velf(2); posf(2)]);
    cost = costst' * P * costst;
    
%     % round(fval)
%     % figure;
%     plot(totalT, fval, 'xr'); hold on;
%     %text(blah, fval, num2str(fval));
%     %text(totalT, cost,       "phi: "   + num2str(banging_phi(blah))+" %");
%     %text(totalT, cost + 100, "theta: " + num2str(banging_theta(blah))+" %");
%     plot(totalT, cost, 'xb');
%     %text(totalT, cost + 200, "T: " + num2str(totalT) + " sec");
%     plot(totalT, banging_phi(blah), 'xg');
%     plot(totalT, banging_theta(blah), 'xg');
%     grid on;
    
    blah = blah + 1;
    %text(blah, cost, num2str(cost));
end
%% STEP4: plot the emulation
% figure;
% plot(t, states(:,:)); hold on;
% text(t(1), states(1,1), 'v_x start');
% text(t(1), states(2,1), 'x start');
% text(t(1), states(3,1), 'v_y start');
% text(t(1), states(4,1), 'y start');
% 
% plot(t, pos);
% figure;
%  plot(t, vel);
% 
% text(t(end),posf(1,1),'desired x');
% text(t(end),posf(1,2),'desired y');
% text(t(end),velf(1,1),'desired v_x');
% text(t(end),velf(1,2),'desired v_y');
% 
% grid on;
% xlabel('time'); ylabel('states');
% legend('v_x', 'x', 'v_y', 'y', 'x nl', 'y nl', 'v_x nl', 'v_y nl');

%%

% figure;
% plot(pos(:,1), pos(:,2)); hold on;
% text(pos(1,1), pos(1,2), 'start');
% plot(posf(1,1), posf(1,2), 'xr');
% plot(states(2,:), states(4,:));
% grid on;  axis equal;
% xlabel('x(m)'); ylabel('y(m)');
% legend('nonlinear','linear prog cost');

%% velocity surface compare

figure;
x1 = pos(:,1);
y1 = pos(:,2);
z1 = absvel(:,1);
% Plot data:
surf([x1(:) x1(:)], [y1(:) y1(:)], [z1(:) z1(:)], ...  % Reshape and replicate data
     'FaceColor', 'none', ...    % Don't bother filling faces with color
     'EdgeColor', 'interp', ...  % Use interpolated color for edges
     'LineWidth', 2);            % Make a thicker line
axis equal;
% text(x1(1), y1(1), 'start');
% text(x1(end), y1(end), 'end');
% xlim([-2.5, 0.5]);
% ylim([0 3]);
xlabel('x'); ylabel('y'); 
view(2);   % Default 2-D view
h = colorbar;
caxis([0 10]);
ylabel(h, 'v_h (m/s)')

%%
set(0, 'DefaultLineLineWidth', 2);
figure;
subplot(2,1,1);
plot(t, phi_arr .* 180/3.142); grid on; hold on;
plot(tkumar, phikumar);
plot(tkumar, 25 * ones(length(tkumar),1), '--r');
plot(tkumar, -25 * ones(length(tkumar),1), '--r');
legend('roll (proposed)', 'roll (min snap)', 'saturation');
xlabel('time'); ylabel('\phi');

subplot(2,1,2);
plot(t, theta_arr .* 180/3.142); grid on; hold on;
plot(tkumar, thetakumar);
plot(tkumar, 25 * ones(length(tkumar),1), '--r');
plot(tkumar, -25 * ones(length(tkumar),1), '--r');
legend('pitch (proposed)', 'pitch (min snap)', 'saturation');
xlabel('time'); ylabel('\theta');