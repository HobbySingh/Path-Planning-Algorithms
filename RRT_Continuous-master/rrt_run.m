clc
clear
close all

environment = load('environment.mat');
x = 5;
delta = 2;
source = [1,147/x];
goal = [230/x,20];
%goal = [150/(5*x),250/(5*x)];
[edges,vertices] = rrt_implement(environment.empty_world, source, goal, (delta)*(250/x)*0.05);
%[edges,vertices] = rrt_start_implement(environment.empty_world, source, goal, 3, 3);