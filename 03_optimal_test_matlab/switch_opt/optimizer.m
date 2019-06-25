function t1 = optimizer(x0, v0, xd, vd)
    

    % t1 = (-x0(1) / v0(1)) * 0.55;
    T = (xd(1) - x0(1))/((v0(1) + vd(1))/2);
    t1 = T * 0.55;
    % t1 = t1 * 2;
    
    figure(14)
    hold off;
    grid on;
    
    maxbank = 35 / 57;
    dT1 = 0.01;
    
    % Gradient descend
    for i=1:1000
        disp([' ------------ Iteration ---------- ' num2str(i)])
        
        % Ref
        c0 = PathPredict(x0, v0, xd, vd, [40/57, -40/57, t1])
        
        % param plus
        c1p = PathPredict(x0, v0, xd, vd, [40/57, -40/57, t1 + dT1])
        
        % param minus
        c1m = PathPredict(x0, v0, xd, vd, [40/57, -40/57, t1 - dT1])
        
        % going too fast! 
        if ((c0 <= c1p) && (c1p <= c1m))
            dT1 = dT1/2;
        end
        if ((c0 <= c1m) && (c1m <= c1p))
            dT1 = -dT1/2; %?
        end
        if ((c1p < c0) && (c0 < c1m))
            t1 = t1 + dT1;
        end
        if ((c1p <= c1m) && (c1m <= c0))
            disp("error? c1p < c1m < c0")
            % t1 = t1 + dT1;
        end
        if ((c1m < c0) && (c0 < c1p))
            t1 = t1 - dT1;
        end
        if ((c1m <= c1p) && (c1p <= c0))
            disp("error? c1m < c1p < c0")
            % t1 = t1 - dT1;
        end
        % kick epsilon here and there?
        if (abs((c1m - c1p) < 0.01) && (abs(c1p - c0) < 0.01))
            t1 = t1 + t1 * 0.01 * (rand(1) - 0.5);
        end
              
        plot(i, c0, 'xb'); hold on;
   
        
    end
        
    grid on
    xlabel('Time [step]')
    ylabel('Cost')

      
    [c x v] = PathPredict(x0, v0, xd, vd, [40.57, -40/57, t1])
    
    %phi0 * 57
    %phi1 * 57
    %t1
    
end