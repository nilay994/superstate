function [cost, yt, vt] = PathPredict(x0, v0, param, t_comp)
    %PathPredict Predict
    %   Detailed explanation goes here

    phi0 = param(1);
    phi1 = param(2);
    t1 = param(3);

%     if (debug)
%         figure(13)
%         hold off
%     end
    
    y = x0;
    v = v0;
    dt = 1/100;
    t = 0;

    i = 1;
    str = zeros(50/dt,1);
    % Predict until passing the gate
    while (t <= t_comp && y < 0)
        phi = phi0;
        if (t>t1)
            phi = phi1;
        end

        if (phi > 1)
            phi = 1;
        elseif (phi < -1)
            phi = -1;
        end

        ay = sin(phi) * 9.81 / cos(phi*0.85) - v * 0.52;

        % Simulation
        t = t + dt;
        v = v + ay .* dt;
        y = y + v * dt; 
        str(i) = y;
        i = i + 1;
    end


    % Position at the Gate
    yt = y;
    vt = v;

    cost = abs(y).^2 + abs(v).^2 * 10;

end

