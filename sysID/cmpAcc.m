close all; clear all; clc;

gt = csvread('tf.csv');
t = gt(:,1);
t = t - t(1);
roll = gt(:,2);
roll_cmd = gt(:,3);
set(0, 'DefaultLineLineWidth', 2);
pitch = gt(:,4)  * 180/3.142;
pitch_cmd = gt(:,5) * 180/3.142;

roll_data = iddata(roll, roll_cmd, mean(gradient(t)));
roll_tf = tfest(roll_data, 2, 0);
compare(roll_tf, roll_data); grid on;

lsim(roll_tf, 35 * ones(1000), linspace(0,2,1000)); 

% roll_tf_d = c2d(roll_tf, mean(gradient(t)));
% roll_ss_d = ss(roll_tf_d);
% sim(roll_tf_d, 10 * 3.142/180 * ones(1001, 1), [0:0.001:1]);
% 
% mew = canon(roll_ss_d,'Companion');
% do I really need UCB format of states? I think I am happier with terminal
% velocities than angles?! what say :P