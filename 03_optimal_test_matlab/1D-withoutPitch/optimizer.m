function [phi0, phi1, t1] = optimizer(y0, v0, t_comp, debug)
yd = 0;
%OPTIMIZER Solve
%   Detailed explanation goes here

% x0 = [-14 4];
% v0 = [5 0];

if (debug)
    figure(14)
    hold off;
    grid on
end

    phi0 = 0;
    phi1 = 0;
    
    v_avg = -(y0-yd)/t_comp;  
    t1 = -(y0-yd)/v_avg * 0.5;  % pitching at 20deg means 4m/s

    dphi0 = 4/57;
    dphi1 = 4/57;
    dt1 = t1 / 100;
    
    % Gradient descend
    for i=1:200
        disp([' ------------ Iteration ---------- ' num2str(i)])
        
        % Ref
        c0 = PathPredict(y0, v0, [phi0, phi1, t1], t_comp)
        
        % Parameter 1
        c1p = PathPredict(y0, v0, [phi0 + dphi0, phi1, t1], t_comp)

        % Parameter 2
        c2p = PathPredict(y0, v0, [phi0, phi1 + dphi1, t1], t_comp)

        % Parameter 3
        
        if (c1p > c0)
            c1m = PathPredict(y0, v0, [phi0 - dphi0, phi1, t1], t_comp)
            if (c1m < c0)
                % Change direction
                dphi0 = -dphi0;
            else
                dphi0 = dphi0 / 2;
                continue;
            end
        end
        if (c2p > c0)
            c2m = PathPredict(y0, v0, [phi0, phi1 - dphi1, t1], t_comp)
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
        
        if (debug)
            plot(i,c0,'xb')
            hold on
        end
        
        if abs(phi0) > (abs(phi1) + 0.05)
            %t1 = t1 + dt1;
        elseif abs(phi1) > (abs(phi0) + 0.05)
            %t1 = t1 - dt1;
        end
            
    end
    
    
%     if debug
%         grid on
%         xlabel('Time [step]')
%         ylabel('Cost []')
%         axis([0 i 0 1])
%     end
%     
%     
    [c yfinal vfinal] = PathPredict(y0, v0, [phi0, phi1, t1], t_comp)
    sprintf("final cost %f \n", c)
    %phi0 * 57
    %phi1 * 57
    %t1
    
 end

