close all; clear all; clc;

gt = csvread('tf.csv');
t = gt(:,1);
t = t - t(1);
roll = gt(:,2);
roll_cmd = gt(:,3);

pitch = gt(:,4);
pitch_cmd = gt(:,5);

roll_data = iddata(roll, roll_cmd, mean(gradient(t)));
roll_tf = tfest(roll_data, 2, 0);
compare(roll_tf, roll_data);

roll_tf_d = c2d(roll_tf, mean(gradient(t)));
roll_ss_d = ss(roll_tf_d);
sim(roll_tf_d, 10 * 3.142/180 * ones(1001, 1), [0:0.001:1]);

mew = canon(roll_ss_d,'Companion');
% do I really need UCB format of states? I think I am happier with terminal
% velocities than angles?! what say :P