%% This script runs the simulation for dcapt

%% Clear the previous
clear 
figure(1);
clf(1);
%% Parameters

% num robots and goals
var.nbots = 10;

% num dimensions
var.n = 2;

% radius
var.R = 0.086;  %** radius is different in some assignment sheet **%

% boundary x axis and y axis
var.bound = 50;

% max vel
var.vmax = 30;

% st time
var.t0 = 0;

% neighb dist
var.h = 3;

%% Assign the goals
[start,goal] = generateStartGoal(var, true);
% start=[0,0;15,0]; goal=[11,6;4,6]; 
var.f = randperm(var.nbots);
% var.f = [1,2];
var.start = start;
var.goal = goal;
var.t_step = 0.01;
var.quadPlot = MyQuadPlot(var.start, var.goal, var.R, var.bound, var.h);
% the radius of the robot
var.scale = 10;

dcaptWrapper(var)

