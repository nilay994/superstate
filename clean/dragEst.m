function [kdx1, kdy1, kdx2, kdy2, kdx3, kdy3] = dragEst(angBody, filt_a, optiAcc, optiVel, rpmAvg, t)

    bodyAcc = zeros(length(t), 3);
    bodyVel = zeros(length(t), 3);
    
    % estimate accelerations and velocity in body frame  
    for i = 2:1:length(t)

        phi = angBody(i,1);
        theta = angBody(i,2);
        psi = angBody(i,3);

        R = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
          sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
          sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
          cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
          cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];

        bodyAcc_t = R * [optiAcc(i,1); optiAcc(i,2); (optiAcc(i,3) - 9.81)]; % add grav here NED
        bodyVel_t = R * [optiVel(i,1); optiVel(i,2); optiVel(i,3)];

        bodyAcc(i,1) = bodyAcc_t(1);
        % bodyAcc(i,1) = filt_a(i,1) + 9.81 * sin(theta); % specific force
        % vs real acceleration
        bodyVel(i,1) = bodyVel_t(1);

        bodyAcc(i,2) = bodyAcc_t(2);
        bodyVel(i,2) = bodyVel_t(2);

        bodyAcc(i,3) = bodyAcc_t(3);
        bodyVel(i,3) = bodyVel_t(3);

    end
    
    
    %% linear drag model, simple
    figure; 
    plot(bodyVel(:,1), bodyAcc(:,1), '.', 'Color', [180/255, 180/255, 180/255]); 
    hold on; grid on;
    [p_x, S] = polyfit(bodyVel(:,1), bodyAcc(:,1), 1);
    yaxis = polyval(p_x, bodyVel(:,1), S);
    plot(bodyVel(:,1), yaxis, '-r', 'LineWidth', 2);
    xlabel('v^B_x (m/s)'); ylabel('a^B_x (m/s^2)');
    r2calc = 1 - (S.normr/norm(bodyAcc(:,1)) - mean(bodyAcc(:,1)))^2;
    title(['basic linear drag model, ', 'R^2 fit of: ', num2str(r2calc)]);
    kdx1 = -p_x(1);


    % drag co-efficient in y direction
    figure;
    plot(bodyVel(:,2), filt_a(:,2), '.', 'Color', [180/255, 180/255, 180/255]); 
    hold on; grid on;
    [p_y, S] = polyfit( bodyVel(:,2), filt_a(:,2), 1);
    yaxis = polyval(p_y, bodyVel(:,2), S);
    plot(bodyVel(:,2), yaxis, '-r', 'LineWidth', 2);
    xlabel('v^B_y (m/s)'); ylabel('a^B_y (m/s^2)');
    r2calc = 1 - (S.normr/norm(filt_a(:,2)) - mean(filt_a(:,2)))^2;
    title(['basic linear drag model, ', 'R^2 fit of: ', num2str(r2calc)]);

    kdy1 = -p_y(1);
    
    
    %% linear drag model, estimate using polyfit and plot, Kumar
    
    % drag co-efficient in x direction
    figure; 
    plot(rpmAvg(:,1).* bodyVel(:,1), filt_a(:,1),'.', 'Color', [180/255, 180/255, 180/255]); 
    hold on; grid on;
    [p_x, S] = polyfit(rpmAvg(:,1).* bodyVel(:,1), filt_a(:,1), 1);
    yaxis = polyval(p_x, rpmAvg(:,1).* bodyVel(:,1), S);
    plot(rpmAvg(:,1).* bodyVel(:,1), yaxis,  '-r', 'LineWidth', 2);
    xlabel('v^B_x * \omega_s (m/s * rad/s)'); ylabel('a^B_x (m/s^2)');
    r2calc = 1 - (S.normr/norm(filt_a(:,1)) - mean(filt_a(:,1)))^2;
    title(['prop speed linear drag model, ', 'R^2 fit of: ', num2str(r2calc)]);

    kdx2 = -p_x(1);


    % drag co-efficient in y direction
    figure;
    plot(rpmAvg(:,1).* bodyVel(:,2), filt_a(:,2),'.', 'Color', [180/255, 180/255, 180/255]); 
    hold on; grid on;
    [p_y, S] = polyfit(rpmAvg(:,1).* bodyVel(:,2), filt_a(:,2), 1);
    yaxis = polyval(p_y, rpmAvg(:,1).* bodyVel(:,2), S);
    plot(rpmAvg(:,1).* bodyVel(:,2), yaxis, '-r', 'LineWidth', 2);
    xlabel('v^B_y * \omega_s (m/s * rad/s)'); ylabel('a^B_y (m/s^2)');
    r2calc = 1 - (S.normr/norm(filt_a(:,2)) - mean(filt_a(:,2)))^2;
    title(['prop speed linear drag model, ', 'R^2 fit of: ', num2str(r2calc)]);

    kdy2 = -p_y(1);
    
    %% towards identification using thrust, suggested by flapping dynamics
    % by Tarek
    T(:,1) = bodyAcc(:,3);
    
    % drag co-efficient in x direction
    figure; 
    plot(T(:,1) .* bodyVel(:,1), filt_a(:,1), '.', 'Color', [180/255, 180/255, 180/255]); 
    hold on; grid on;
    [p_x, S] = polyfit(T(:,1).* bodyVel(:,1), filt_a(:,1), 1);
    yaxis = polyval(p_x, T(:,1).* bodyVel(:,1), S);
    plot(T(:,1).* bodyVel(:,1), yaxis, '-r', 'LineWidth', 2);
    xlabel('v^B_x * thrust (m/s * N)'); ylabel('a^B_x (m/s^2)');
    r2calc = 1 - (S.normr/norm(filt_a(:,1)) - mean(filt_a(:,1)))^2;
    title(['thurst linear drag model, ', 'R^2 fit of: ',  num2str(r2calc)]);

    kdx3 = -p_x(1);


    % drag co-efficient in y direction
    figure;
    plot(T(:,1).* bodyVel(:,2), filt_a(:,2), '.', 'Color', [180/255, 180/255, 180/255]); 
    hold on; grid on;
    [p_y, S] = polyfit(T(:,1).* bodyVel(:,2), filt_a(:,2), 1);
    yaxis = polyval(p_y, T(:,1).* bodyVel(:,2), S);
    plot(T(:,1).* bodyVel(:,2), yaxis, '-r', 'LineWidth', 2);
    xlabel('v^B_y * thrust (m/s * N)'); ylabel('a^B_y (m/s^2)');
    r2calc = 1 - (S.normr/norm(filt_a(:,2)) - mean(filt_a(:,2)))^2;
    title(['thrust linear drag model, ', 'R^2 fit of: ',  num2str(r2calc)]);

    kdy3 = -p_y(1); 
    
%%  

    % filter the body accelerations  
    % 2Hz cutoff, 5th order bessel filter
    filter_acc = mkfilter(2, 5, 'bessel');
    filt_opti(:,1) = lsim(filter_acc, bodyAcc(:,1), t);
    filt_opti(:,2) = lsim(filter_acc, bodyAcc(:,2), t);
    filt_opti(:,3) = lsim(filter_acc, bodyAcc(:,3), t);

    % verify co-incide - better without kdx*2, however dead reckoned 
    % position is much better with kdx*2; suspecting lumped identification bug 
    figure; 
    subplot(2,1,1);
    plot(t, filt_opti(:,1)); hold on; grid on;
    plot(t, filt_a(:,1));
    % plot(t, -kdx3 * T(:,1).* bodyVel(:,1));
    plot(t, -kdx1 * bodyVel(:,1));
    legend('a_x (optiTrack)', 'filt_{ax}', 'drag model');
    title('lateral x acceleration, body frame');

    subplot(2,1,2);
    plot(t, filt_opti(:,2)); hold on; grid on; 
    plot(t, filt_a(:,2)); 
    plot(t, -kdy3 * T(:,1).* bodyVel(:,2));
    legend('a_y (optiTrack)', 'filt_{ay}', 'drag model');
    title('lateral y acceleration, body frame');
    

end






%     %% accelerometer time!! 
%     figure; 
%     plot(bodyVel(:,1), filt_a(:,1), '.', 'Color', [180/255, 180/255, 180/255]); 
%     hold on; grid on;
%     [p_x, S] = polyfit(bodyVel(:,1), filt_a(:,1), 1);
%     yaxis = polyval(p_x, bodyVel(:,1), S);
%     plot(bodyVel(:,1), yaxis, '-r', 'LineWidth', 2);
%     xlabel('v^B_x (m/s)'); ylabel('a^B_x (m/s^2)');
%     r2calc = 1 - (S.normr/norm(filt_a(:,1)) - mean(filt_a(:,1)))^2;
%     title(['optiTrack - basic linear drag model using acc, ', 'R^2 fit of: ', num2str(r2calc)]);
%     kdx1 = -p_x(1);
% 
% 
%     % drag co-efficient in y direction
%     figure;
%     plot(bodyVel(:,2), filt_a(:,2), '.', 'Color', [180/255, 180/255, 180/255]); 
%     hold on; grid on;
%     [p_y, S] = polyfit( bodyVel(:,2), filt_a(:,2), 1);
%     yaxis = polyval(p_y, bodyVel(:,2), S);
%     plot(bodyVel(:,2), yaxis, '-r', 'LineWidth', 2);
%     xlabel('v^B_y (m/s)'); ylabel('a^B_y (m/s^2)');
%     r2calc = 1 - (S.normr/norm(filt_a(:,2)) - mean(filt_a(:,2)))^2;
%     title(['optiTrack - basic linear drag model using acc, ', 'R^2 fit of: ', num2str(r2calc)]);
% 
%     kdy1 = -p_y(1);
    
    