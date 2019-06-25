function [cost, xt, vt] = PathPredict(x0, v0, xd, vd, param)

    flap = 1; %1
    phi0 = param(1);
    phi1 = param(2);
    t1 = param(3);
    psi = 0;

    x = x0;
    v = v0;
    dt = 1/100;
    t = 0;

    maxbank = 45 / 57;
    K_ff_theta = 14/57 / 5;   % rad to fly at (e.g. 10 deg = 5 m/s)
    K_p_theta = 6 / 57;       % m/s to radians

    % Predict until passing the gate
    while ((x(1) - xd(1)) < 0.1)  % TODO: don't forget to change the directions

        % Control
        theta = (vd(1) - v(1)) * K_p_theta + vd(1) .* K_ff_theta; 
        if (theta > maxbank)
            theta = maxbank;
        elseif (theta < -maxbank)
            theta = -maxbank;
        end 
        
        phi = phi0;
        if (t>t1)
            phi = phi1;
        end

        T =  9.81 / (cos(phi * flap) * cos(theta * flap));
        ax = (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * T - v(1) * 0.56;
        ay = (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * T - v(2) * 0.56;

        % Simulation
        t = t + dt;
        dv = [ax, ay];
        v = v + dv .* dt;
        x = x + v .* dt;

        % plot(x(1),x(2),'xb'); hold on;

    end

    % Position at the Gate
    xt = x;
    vt = v;

    cost = abs(x(2)-xd(2)).^2 * 10 + abs(vd(2)-v(2)).^2;
end