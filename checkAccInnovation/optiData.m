% filtering in higher order causes loss of data, 
% integrating it back introduces drift
    
function [filt_a, optiAcc, optiVel] = optiData(optiPos, accel, t, dt, n, st)
    
    % apply filtering from both sides. filter(b,a) is forward mean only,
    % which introduces delay. However, the vector head and tail are handled
    % better in filter(b,a)
    optiVel(:,1) = smooth(gradient(optiPos(:,1))/dt, n);
    optiVel(:,2) = smooth(gradient(optiPos(:,2))/dt, n);
    optiVel(:,3) = smooth(gradient(optiPos(:,3))/dt, n);

    optiAcc(:,1) = smooth(gradient(optiVel(:,1))/dt, n);
    optiAcc(:,2) = smooth(gradient(optiVel(:,2))/dt, n);
    optiAcc(:,3) = smooth(gradient(optiVel(:,3))/dt, n);
    
    % smooth peaks, kick em out
    optiAcc(1:st, :) = 0;
    optiVel(1:st, :) = 0;
    
    idx = round(1.5*n);
    optiAcc(end-idx:end, :) = 0;
    optiVel(end-idx:end, :) = 0;
    % be ready for delay if using filter(b,a)...
%         windowSize = n; 
%         b = (1/windowSize)*ones(1,windowSize);
%         a = 1;
      
%     optiVel(:,1) = filter(b,a, gradient(optiPos(:,1))/dt);
%     optiVel(:,2) = filter(b,a, gradient(optiPos(:,2))/dt);
%     optiVel(:,3) = filter(b,a, gradient(optiPos(:,3))/dt);
%     
%     optiAcc(:,1) = filter(b,a, gradient(optiVel(:,1))/dt);
%     optiAcc(:,2) = filter(b,a, gradient(optiVel(:,2))/dt);
%     optiAcc(:,3) = filter(b,a, gradient(optiVel(:,3))/dt);
    

    % filter the body accelerations  
    % 2Hz cutoff, 5th order bessel filter
    filter_acc = mkfilter(2, 5, 'bessel');
    filt_a(:,1) = lsim(filter_acc, accel(:,1), t);
    filt_a(:,2) = lsim(filter_acc, accel(:,2), t);
    filt_a(:,3) = lsim(filter_acc, accel(:,3), t);
    
    
end
