
%% METHOD 3: also use gyroscopic terms, moments EKF time

    % trivial thrust model, less information seeping into lateral
    % directions, might work if flights aren't agile. comment out if back
    % to precise thrust, after thrust modelling
    % thrust(3) = -9.81/(cos(theta)*cos(phi));

% METHOD1: Convert filtered accelerometer to world frame and dead reckon
    % suggestion: maybe yaw drift is propagating reference: 
    % Kumar: Inertial Yaw-Independent Velocity and Attitude Estimation for High-Speed Quadrotor Flight
    
%     acc = R' * accel(i,:)';
%     % remove pseudo gravity
%     acc(3) = acc(3) + g;
% % accelerometer also experiences the force due to drag, and it must be
% % compensated in the inertial frame?
%     acc(1) = acc(1);% - kdx1 * vel_w(i-1,1);
%     acc(2) = acc(2);% - kdx2 * vel_w(i-1,2);
%     acc_w(i,1:3) = acc';
%     vel_w(i,1:3) = vel_w(i-1,1:3) + (acc .* dt)';
%     pos_w(i,1:3) = pos_w(i-1,1:3) + vel_w(i,1:3) .* dt;   

      % complementary observer gain - kumar 2012
%       kw = -0.08;
%       thrust = R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)]; 
%       P = [1 0 0; 0 1 0];
%       kD = [-kdx3 0 0; 0 -kdy3 0; 0 0 0];
%       a_body = thrust(3) * kD * R * P' * [vel_w(i-1,1); vel_w(i-1,2)];
%       velHat = inv(thrust(3) * P * R' * kD * R * P') * ([optiAcc(i,1); optiAcc(i,2)] + (P * R' * [0;0;thrust(3)]));
%       % observer for velocity - nullify the bias
%       acc_t = ([0;0;9.81] + R'* [0;0;thrust(3)] + R' * [a_body(1); a_body(2); 0] + kw * (vel_w(i-1,:)' - [velHat; 0]))';
%       acc_w(i,1) = acc_t(1);
%       acc_w(i,2) = acc_t(2);
%       acc_w(i,3) = 0;
%       vel_w(i,1:3) = vel_w(i-1,1:3) + (acc_w(i,1:3) .* dt);
%       pos_w(i,1:3) = pos_w(i-1,1:3) + (vel_w(i,1:3) .* dt);
          


    % METHOD2: Make use of drag co-efficients
%     acc = R' * [0; 0; -g/(cos(theta) * cos(phi))];
%     acc(3) = acc(3) + g;
%     % drag model
%     acc_w2(i,1) = acc(1) - vel_w2(i-1,1) * kdx_old;
%     acc_w2(i,2) = acc(2) - vel_w2(i-1,2) * kdy_old;
%     acc_w2(i,3) = acc(3);
%     
%     vel_w2(i,1:3) = vel_w2(i-1,1:3) + (acc .* dt)';
%     pos_w2(i,1:3) = pos_w2(i-1,1:3) + vel_w2(i,1:3) .* dt;


    % filter 3, assume accelerations are known, use thrust from optiTrack augment with gravity
    % there is no biasless 9.81. Calibration error in
    % accelerometer/temperature bias will have to be dealt with. no bias
    % term in thrust model? be ready to drift in velocity estimates.
    % maybe depends on the quality of tracking. use more markers, less
    % noise, better integration, 2019-04-26_20_43_06.csv shows no
    % drift in position and perfect coherance with ground truth
%     acc = R' * [thrust(1); thrust(2); thrust(3)];
%     % drag model
%     acc_w3(i,1) = acc(1);
%     acc_w3(i,2) = acc(2); 
%     acc_w3(i,3) = acc(3) + 9.81;
%     
%     vel_w3(i,1:3) = vel_w3(i-1,1:3) + (acc_w3(i,:) .* dt);
%     pos_w3(i,1:3) = pos_w3(i-1,1:3) + vel_w3(i,1:3) .* dt; %+ 0.5 .* acc_w3(i,1:3) .* dt .* dt;
%     

%     % with centrifugal forces 
%     kd = rpmAvg(i,1) * [-kdx1 0 0; 0 -kdy1 0; 0 0 0];
%     a_body = (kd * R * optiVel(i-1, 1:3)')'; % plus some bias. 
%     
%     phi_d = (phi - phi_prev)/dt;
%     theta_d = (theta - theta_prev)/dt;
%     psi_d = (psi - psi_prev)/dt;
%     
%     phi_prev = phi;
%     theta_prev = theta;
%     psi_prev = psi;
%     
%     R_omega = [1, 0, -sin(theta); 
%                0, cos(phi), sin(phi) * cos(theta);
%                0, -sin(phi), cos(phi) * cos(theta)];
%     % R_omega = eye(3);
%     omega = (R_omega) * [phi_d; theta_d; psi_d];
%     P = [1 0 0; 0 1 0; 0 0 0];
%     a_body = a_body - (P * cross(omega, R * optiVel(i-1,1:3)'))';
%     acc_t = ([0;0;9.81] + R'* ([0;0;thrust(3)] + [a_body(1); a_body(2); a_body(3)]))'; % a_body(3) = 0;
%     % acc_t = acc_t - (cross(omega, optiVel(i,:)));
%     acc_w3(i,1) = acc_t(1);
%     acc_w3(i,2) = acc_t(2);
%     acc_w3(i,3) = acc_t(3);
%     vel_w3(i,1:3) = vel_w3(i-1,1:3) + (acc_w3(i,1:3) .* dt);
%     pos_w3(i,1:3) = pos_w3(i-1,1:3) + (vel_w3(i,1:3) .* dt); % + 0.5 .* acc_w2(i,1:3) .* dt .* dt; 



%%

% doubts, 
% why is it in body frame - YingFu resolved the components for you
% can it be also in world frame psuedo force (inertial frame)
% try out propagation and compare pred with GT (problem in Z position)
% Why isn't body thrust of g/(cos*cos) used in drag estimation?
% is the sensor and body CoM on same axes
% apply 5th order Bessel filter, 2Hz cutoff
% obviously I can't dead reckon the accelerations: body to world
% accelerations, double integration, wouldn't work. Vibration sensor is
% noisy. 
% We have three types of body accelerations. 
% 1) rotated double derivative optitrack.
% 2) accelerometer body measurements
% 3) observer design: predict accelerations using AHRS and thrust profile.

% Why are we using the velocity in the world frame and multiplying it with
% the drag? read a few papers? :( - SiHao Sun, YingFu
% todo: Faessler uzh-rpg/drone-controller document for math and rigid body dynamics 
% todo: use Kdrag.m equations here using dead reckoning of accelerometer
% readings (DONE)
% todo: try Shuo's equations in Kdrag.m (DONE)
% todo: try Shou's equations here (DONE)
% todo: shuo says control in drag is already included. (thrust component isn't -
% Yingfu)
% todo: compare zurich ROS and UPenn kumar to understand their strategies
% todo: WRITE!! on sticky with printouts of important papers
% todo: take meeting next week
% doubt: No thrust (body z) compensation for drag? 
% is the hypothesis correct? There is a drift in the AHRS and hence there
% is drift in the filter?  No. There are drag forces and external
% disturbances. If the EoM are integrated using Euler (Kumar) do we expect
% a better result due to "virtual coupling" or do we call "dual quaternions" for
% rescue
% todo: Check gyroscopic effects
% minimum pitch angle so that the wake of one doesn't affect the other,
% maybe better in trashcan than bebop
% check how inertial conservative forces (body resolve drag) works
% todo: check drag coefficient only from IMU - (FAILS)
% todo: need to check low pass filter method (which could also mean an integrator).
% It will surely give non zero estimated velocities. It is not a problem
% for the time being

% interesting expt: Pitching produces acceleration in x axis. However it
% seems that the drone doesn't accelerate, but keeps going in constant
% velocity while pitching. This would imply that there are some forces that
% are making the net lateral acceleration = 0. Hence we equate the drag
% force against thrust force. However to try:
% 1) paparazzi: constant pitch
% 2) see if acceleration in x axis via optiTrack

% find out how much yaw drift causing drift in actual model
% find out if 9.81 accelerometer when stationary to know more about biases:
% Yes, it throws -8.8 wtf!, need to do something about it. AHRS calib wiki 
%% Peter Corke book
ax=10;
ay=20;
az=30;

Cx = [1 0 0; 0 cosd(ax) -sind(ax); 0 sind(ax) cosd(ax)];
Cy = [cosd(ay) 0 sind(ay); 0 1 0; -sind(ay) 0 cosd(ay)];
Cz = [cosd(az) -sind(az) 0; sind(az) cosd(az) 0; 0 0 1];


%% earth to body shuo

    R_E_B = [cos(COMP.THETA(i))*cos(COMP.PSI(i)) cos(COMP.THETA(i))*sin(COMP.PSI(i)) -sin(COMP.THETA(i));...
        sin(COMP.PHI(i))*sin(COMP.THETA(i))*cos(COMP.PSI(i))-cos(COMP.PHI(i))*sin(COMP.PSI(i))...
        sin(COMP.PHI(i))*sin(COMP.THETA(i))*sin(COMP.PSI(i))+cos(COMP.PHI(i))*cos(COMP.PSI(i)) sin(COMP.PHI(i))*cos(COMP.THETA(i));...
        cos(COMP.PHI(i))*sin(COMP.THETA(i))*cos(COMP.PSI(i))+sin(COMP.PHI(i))*sin(COMP.PSI(i))...
        cos(COMP.PHI(i))*sin(COMP.THETA(i))*sin(COMP.PSI(i))-sin(COMP.PHI(i))*cos(COMP.PSI(i)) cos(COMP.PHI(i))*cos(COMP.THETA(i))];

%% integrate optiTrack back
op_acc = zeros(length(t), 3);
op_vel = zeros(length(t), 3);
op_pos = zeros(length(t), 3);

op_vel(1,1:3) = optiVel(1,1:3);
op_pos(1,1:3) = optiPos(1,1:3);

for i=2:1:length(t)
    op_acc(i-1,1:3) = optiAcc(i-1,1:3);
    op_vel(i,1:3) = op_vel(i-1,1:3) + op_acc(i-1,1:3)*dt;
    op_pos(i,1:3) = op_pos(i-1,1:3) + op_vel(i-1,1:3)*dt + 0.5*op_acc(i-1,1:3)*(dt^2);
end
figure;
plot3(op_pos(:,1), op_pos(:,2), op_pos(:,3), '.b'); 
hold on; grid on; axis equal;
plot3(optiPos(:,1), optiPos(:,2), optiPos(:,3), '.r');

figure;
subplot(2,1,1);
plot(t, op_vel(:,1)); hold on; grid on;
plot(t, optiVel(:,1));
subplot(2,1,2);
plot(t, op_vel(:,2)); hold on; grid on;
plot(t, optiVel(:,2));

%%

x = -5:0.0001:5;
y = m*x + c + rand(1, length(x));
 
for i = 1:1:length(x)
    A(i,1) = 1;
    A(i,2) = x(i);
end

params = A\y';

for i = 1:1:length(x)
    newY(i) = A(i,:) * params;
end
plot(x,y,'.'); hold on; grid on;
plot(x,newY); hold on; grid on;
histogram(y-newY, 'Normalization', 'pdf')
