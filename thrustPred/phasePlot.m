


%% solve symbolics
syms vi;
% syms vh; 
% syms vinf;
% syms aoa;
vinf_t = -5:0.1:5;
aoa_t = 6 * vinf_t;
vh = sqrt(0.981/(2*1.225*0.0312));
vinf = 5;
aoa = 30;
eqn = vi == vh^2 / (sqrt((vinf*cos(aoa))^2 + (vi-vinf*sin(aoa))^2));
[solx, param, cond] = solve(eqn, vi, 'ReturnConditions', true)

% for i = 1:1:length(vinf_t)
%     vinf = vinf_t(i);
%     aoa = aoa_t(i);
%     eqn = vi == vh^2 / (sqrt((vinf*cos(aoa))^2 + (vi-vinf*sin(aoa))));
%     sol(i) = real(double(solve(eqn, vi)));
%     ratio(i) = (sol(i) - vinf*sin(aoa))/vh;
% end
%%
% [vinf_t, aoa_t, ratio] = peaks;
% % ratio = 
% contour(vinf_t, aoa_t, ratio)
% 
% %%
% syms vi;
% 
% vinf = 0;
% aoa = 0;
% vh = sqrt(0.981/(2*1.225*0.0312));
% eqn = vi == vh^2 / (sqrt((vinf*cos(aoa))^2 + (vi-vinf*sin(aoa))));
% double(solve(eqn, vi))

%% y = c^2 / (sqrt((b*cos(x))^2 + (y-b*sin(x))^2)) wolfram