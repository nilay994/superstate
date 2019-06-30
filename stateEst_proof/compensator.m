function [compensated_acc, par_x, par_y, par_z] = compensator(grndtruth, accel, t)

    st = 1;
    %% train
    % trend and bias
    % for body x
    A = zeros(length(t),3);
    x_arr = zeros(length(t),1);

    for i =  st:1:length(t)
        A(i,:) = [1, t(i), t(i)^2];
        x_arr(i,1) = grndtruth(i,1) - accel(i,1);
    end
    par_x = A\x_arr;

    % for body y
    B = zeros(length(t),3);
    y_arr = zeros(length(t),1);

    for i =  st:1:length(t)
        B(i,:) = [1, t(i), t(i)^2];
        y_arr(i,1) = grndtruth(i,2) - accel(i,2);
    end
    par_y = B\y_arr;

    % for body z
    C = zeros(length(t),3);
    z_arr = zeros(length(t),1);

    for i =  st:1:length(t)
        C(i,:) = [1, t(i), t(i)^2];
        z_arr(i,1) = grndtruth(i,3) - accel(i,3);
    end
    par_z = C\z_arr;

    %%  reproject 
    % reproject x
    compensated_acc = zeros(length(t), 3);
    for i =  st:1:length(t)
        error = A(i,:) * par_x;
        compensated_acc(i,1) = accel(i,1) + error;
    end

    figure;
    subplot(3,1,1);
    plot(t, grndtruth(:,1)); hold on;
    plot(t, accel(:,1));
    plot(t, compensated_acc(:,1));

    % reproject y
    for i =  st:1:length(t)
        error = B(i,:) * par_y;
        compensated_acc(i,2) = accel(i,2) + error;
    end

    subplot(3,1,2);
    plot(t, grndtruth(:,2)); hold on;
    plot(t, accel(:,2));
    plot(t, compensated_acc(:,2));

    % reproject z
    for i =  st:1:length(t)
        error = C(i,:) * par_z;
        compensated_acc(i,3) = accel(i,3) + error;
    end

    subplot(3,1,3);
    plot(t, grndtruth(:,3)); hold on;
    plot(t, accel(:,3));
    plot(t, compensated_acc(:,3));

    legend('GT', 'accel', 'accel_{comp}');
end

