clear all; close all; clc;

opt = csvread('plotopt.csv', 1, 0);
pid = csvread('plotpid.csv', 1, 0);

dt = mean(gradient(pid(:,1)));
n = 100;

pidt = pid(:,1)-pid(1,1);
pidx = pid(:,2);
pidy = pid(:,3);

pidvelx(:,1) = smooth(gradient(pidx(:,1))/dt, n);
pidvely(:,1) = smooth(gradient(pidy(:,1))/dt, n);

optt = opt(:,1)-opt(1,1);
optx = opt(:,2);
opty = opt(:,3);

optvelx(:,1) = smooth(gradient(optx(:,1))/dt, n);
optvely(:,1) = smooth(gradient(opty(:,1))/dt, n);

set(0, 'DefaultLineLineWidth', 2);
topplot = figure;
sgtitle('top view, position');
plot(pidx, pidy);
axis equal;
hold on; grid on;
plot(optx, opty);
xlabel('x'); ylabel('y');
plot([-0.37, -0.37],[-12.99827, -11.45827], '-k');
legend('pid','proposed','gate');
saveas(topplot, 'topplot.eps', 'epsc');

posplot = figure;
sgtitle('position');
subplot(2,1,1);
plot(pidt, pidx);
hold on; grid on;
plot(optt, optx);
plot(pidt, -0.37*ones(length(pidt),1), '-k');
xlabel('t'); ylabel('x');
legend('pid','proposed','gate reference');

subplot(2,1,2);
plot(pidt, pidy);
hold on; grid on;
plot(optt, opty);
plot(pidt, -12.23*ones(length(pidt),1), '-k');
xlabel('t'); ylabel('y');
legend('pid','proposed','gate reference');
saveas(posplot, 'posplot.eps', 'epsc');

velplot = figure;
sgtitle('velocity');
subplot(2,1,1);
plot(pidt, pidvelx);
hold on; grid on;
plot(optt, optvelx);
plot(pidt, 4*ones(length(pidt),1), '-k');
xlabel('t'); ylabel('x');
legend('pid','proposed','gate reference');

subplot(2,1,2);
plot(pidt, pidvely);
hold on; grid on;
plot(optt, optvely);
plot(pidt, 0*ones(length(pidt),1), '-k');
xlabel('t'); ylabel('y');
legend('pid','proposed','gate reference');
saveas(velplot, 'velplot.eps', 'epsc');
