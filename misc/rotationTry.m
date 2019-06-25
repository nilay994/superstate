syms d;

R = [cos(d) sin(d); -sin(d) cos(d)];

points = rand(2,100);

plot(points(1,:), points(2,:), '.b');
xlim([-2 2]); ylim([-2 2]);
hold on;

for i=-10:-10:-30
    Rmat = eval(subs(R, d, i*3.142/180));
    new_points = Rmat*points;
    plot(new_points(1,:), new_points(2,:), '.r');
    drawnow;
    pause(0.2);
end