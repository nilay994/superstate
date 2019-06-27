function [phi0, phi1, t1] = optimizer(x0, v0, xd, vd)
    

    % t1 = (-x0(1) / v0(1)) * 0.55;
    t1 = (xd(1) - x0(1))/((v0(1) + vd(1))/2) * 0.45;
    
    
    figure(14)
    hold off;
    grid on;

    phi0 = 0;
    phi1 = 0;
    
    maxbank = 35 / 57;

    dphi0 = 4/57;
    dphi1 = 4/57;
    
    % Gradient descend
    for i=1:1000
        disp([' ------------ Iteration ---------- ' num2str(i)])
        
        % Ref
        c0 = PathPredict(x0, v0, xd, vd, [phi0, phi1, t1])
        
        % Parameter 1
        c1p = PathPredict(x0, v0, xd, vd, [phi0 + dphi0, phi1, t1])

        % Parameter 2
        c2p = PathPredict(x0, v0, xd, vd, [phi0, phi1 + dphi1, t1])

        % Parameter 3
        
        if (c1p > c0)
            c1m = PathPredict(x0, v0, xd, vd, [phi0 - dphi0, phi1, t1])
            if (c1m < c0)
                % Change direction
                dphi0 = -dphi0;
            else
                dphi0 = dphi0 / 2;
                continue;
            end
        end
        if (c2p > c0)
            c2m = PathPredict(x0, v0, xd, vd, [phi0, phi1 - dphi1, t1])
            if (c2m < c0)
                dphi1 = -dphi1;
            else
                dphi1 = dphi1 / 2;
                continue;
            end
        end
        
        if (c1p < c2p)
            phi0 = phi0 + dphi0;
        else
            phi1 = phi1 + dphi1;
        end
        
        plot(i,c0,'xb'); hold on;

        if abs(phi0) > (abs(phi1) + 0.05)
            %t1 = t1 + dt1;
        elseif abs(phi1) > (abs(phi0) + 0.05)
            %t1 = t1 - dt1;
        end
        
        if (phi0 > maxbank)
            phi0 = maxbank;
        end
        if (phi0 < -maxbank)
            phi0 = -maxbank;
        end
        
        if (phi1 > maxbank)
            phi1 = maxbank;
        end
        if (phi1 < -maxbank)
            phi1 = -maxbank;
        end        
        
    end
        
    grid on
    xlabel('Time [step]')
    ylabel('Cost')

      
    [c x v] = PathPredict(x0, v0, xd, vd, [phi0, phi1, t1])
    
    %phi0 * 57
    %phi1 * 57
    %t1
    
end