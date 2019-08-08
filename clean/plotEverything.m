function plotEverything(angBody, optiPos, optiVel, optiAcc, t)
    figure;
    subplot(4,3,1);
    plot(t, angBody(:,2)*180/3.142);
    title('\theta');
    xlabel('secs');

    subplot(4,3,2);
    plot(t, angBody(:,1)*180/3.142);
    title('\phi');
    xlabel('secs');

    subplot(4,3,3);
    plot(t, angBody(:,3)*180/3.142);
    title('\psi');
    xlabel('secs');

    subplot(4,3,4);
    plot(t, optiPos(:,1));
    title('optiPos_x');
    xlabel('secs');

    subplot(4,3,5);
    plot(t, optiPos(:,2));
    title('optiPos_y');

    subplot(4,3,6);
    plot(t, optiPos(:,3));
    title('optiPos_z');

    subplot(4,3,7);
    plot(t, optiVel(:,1));
    title('optiVel_x');

    subplot(4,3,8);
    plot(t, optiVel(:,2));
    title('optiVel_y');

    subplot(4,3,9);
    plot(t, optiVel(:,3));
    title('optiVel_z');

    subplot(4,3,10);
    plot(t, optiAcc(:,1));
    title('optiAcc_x');

    subplot(4,3,11);
    plot(t, optiAcc(:,2));
    title('optiAcc_y');

    subplot(4,3,12);
    plot(t, optiAcc(:,3));
    title('optiAcc_z');
    
    figure;
    % use body angles to rotate body acc to world acc
    % table exerts one g upwards, while IMUz faces downward
    T = -9.81;% - g./(cos(angBody(:,2)).*cos(angBody(:,1)));
    subplot(3,1,1);
    wRb_ax = T .* (cos(angBody(:,1)).*sin(angBody(:,2)).*cos(angBody(:,3)) + sin(angBody(:,1)).*sin(angBody(:,3)));
    plot(t, optiAcc(:,1)); hold on; grid on;
    plot(t, wRb_ax); xlabel('time'); ylabel('acc (m/s^2)');
    legend('acc_x^E, optiTrack', 'R_B^E acc_x^B, ahrs imu');
    title('droneAcc_x in world frame');

    subplot(3,1,2);
    wRb_ay = T .* (cos(angBody(:,1)).*sin(angBody(:,2)).*sin(angBody(:,3)) - sin(angBody(:,1)).*cos(angBody(:,3)));
    plot(t, optiAcc(:,2)); hold on; grid on;
    plot(t, wRb_ay); xlabel('time'); ylabel('acc (m/s^2)');
    legend('acc_y^E, optiTrack', 'R_B^E acc_y^B, ahrs imu');
    title('droneAcc_y in world frame');

    subplot(3,1,3);
    wRb_az = T .* cos(angBody(:,1)).* cos(angBody(:,2));
    plot(t, optiAcc(:,3)); hold on; grid on;
    plot(t, wRb_az + 9.81); xlabel('time'); ylabel('acc (m/s^2)');
    legend('acc_z^E, optiTrack', 'R_B^E acc_z^B, ahrs imu');
    title('droneAcc_z in world frame');
    sgtitle('inertial accelerations from optiTrack v/s accelerometer assuming hover');
    
 end