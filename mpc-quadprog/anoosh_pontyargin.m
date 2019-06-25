clc
clear all
close all
%%
x0 = [-0.5;0] ;
xf = [0;0] ;

x(1) = x0(1) ;
v(1) = x0(2) ;

x2(1) = xf(1) ;
v2(1) = xf(2) ;

delt = 0.01 ;
maxiter = 2000 ;

amax = 1 ;

for idx = 1:maxiter
    x(idx+1) = x(idx) + v(idx)*delt + 0.5*amax*delt^2;
    v(idx+1) = v(idx) + amax*delt;
    if abs(x(idx+1)-xf(1))< 0.005
        break
    end
end

for idx = 1:maxiter
    x2(idx+1) = x2(idx) - v2(idx)*delt + 0.5*amax*delt^2;
    v2(idx+1) = v2(idx) + amax*delt ;
    if abs(x2(idx+1)-x0(1))< 0.005
        break
    end
end


plot(x,v,'*')
hold on
plot(x2,v2,'*')
