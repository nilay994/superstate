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
    p_x = polyfit(bodyVel(:,1), bodyAcc(:,1), 1);
    yaxis = polyval(p_x, bodyVel(:,1));
    plot(bodyVel(:,1), yaxis, '-r', 'LineWidth', 2);
    xlabel('vel^B_x (m/s)'); ylabel('acc^B_x (m/s^2)');
    title('OptiTrack vel_x vs acc in body frame');

    kdx1 = -p_x(1);


    % drag co-efficient in y direction
    figure;
    plot(bodyVel(:,2), bodyAcc(:,2), '.', 'Color', [180/255, 180/255, 180/255]); 
    hold on; grid on;
    p_y = polyfit( bodyVel(:,2), bodyAcc(:,2), 1);
    yaxis = polyval(p_y, bodyVel(:,2));
    plot(bodyVel(:,2), yaxis, '-r', 'LineWidth', 2);
    xlabel('vel^B_y (m/s)'); ylabel('acc^B_y (m/s^2)');
    title('OptiTrack vel_y vs acc in body frame');

    kdy1 = -p_y(1);
    
    
    %% linear drag model, estimate using polyfit and plot, Kumar
    
    % drag co-efficient in x direction
    figure; 
    plot(rpmAvg(:,1).* bodyVel(:,1), bodyAcc(:,1), '.'); 
    hold on; grid on;
    p_x = polyfit(rpmAvg(:,1).* bodyVel(:,1), bodyAcc(:,1), 1);
    yaxis = polyval(p_x, rpmAvg(:,1).* bodyVel(:,1));
    plot(rpmAvg(:,1).* bodyVel(:,1), yaxis);
    xlabel('vel^B_x * avgRPM (m/s * rad/s)'); ylabel('acc^B_x (m/s^2)');
    title('OptiTrack vel vs acc in body frame');

    kdx2 = -p_x(1);


    % drag co-efficient in y direction
    figure;
    plot(rpmAvg(:,1).* bodyVel(:,2), bodyAcc(:,2), '.'); 
    hold on; grid on;
    p_y = polyfit(rpmAvg(:,1).* bodyVel(:,2), bodyAcc(:,2), 1);
    yaxis = polyval(p_y, rpmAvg(:,1).* bodyVel(:,2));
    plot(rpmAvg(:,1).* bodyVel(:,2), yaxis);
    xlabel('vel^B_y * avgRPM (m/s * rad/s)'); ylabel('acc^B_y (m/s^2)');
    title('OptiTrack vel_y vs acc in body frame');

    kdy2 = -p_y(1);
    
    %% towards identification using thrust, suggested by flapping dynamics
    % by Tarek
    T(:,1) = bodyAcc(:,3);
    
    % drag co-efficient in x direction
    figure; 
    plot(T(:,1) .* bodyVel(:,1), bodyAcc(:,1), '.'); 
    hold on; grid on;
    p_x = polyfit(T(:,1).* bodyVel(:,1), bodyAcc(:,1), 1);
    yaxis = polyval(p_x, T(:,1).* bodyVel(:,1));
    plot(T(:,1).* bodyVel(:,1), yaxis);
    xlabel('vel^B_x * thrust (m/s * N)'); ylabel('acc^B_x (m/s^2)');
    title('OptiTrack vel vs acc in body frame');

    kdx3 = -p_x(1);


    % drag co-efficient in y direction
    figure;
    plot(T(:,1).* bodyVel(:,2), bodyAcc(:,2), '.'); 
    hold on; grid on;
    p_y = polyfit(T(:,1).* bodyVel(:,2), bodyAcc(:,2), 1);
    yaxis = polyval(p_y, T(:,1).* bodyVel(:,2));
    plot(T(:,1).* bodyVel(:,2), yaxis);
    xlabel('vel^B_y * thrust (m/s * N)'); ylabel('acc^B_y (m/s^2)');
    title('OptiTrack vel_y vs acc in body frame');

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
    plot(t, -kdx3 * T(:,1).* bodyVel(:,1));
    legend('a_x (optiTrack)', 'filt_{ax}', 'drag model');
    title('lateral x acceleration, body frame');

    subplot(2,1,2);
    plot(t, filt_opti(:,2)); hold on; grid on; 
    plot(t, filt_a(:,2)); 
    plot(t, -kdy3 * T(:,1).* bodyVel(:,2));
    legend('a_y (optiTrack)', 'filt_{ay}', 'drag model');
    title('lateral y acceleration, body frame');
    

end