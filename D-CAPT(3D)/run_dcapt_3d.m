%% Main with simulation

%% Clear the previous
clear 
clear dcaptTrajGenerator
close all
clc
%% Parameters
disp('Planning ...');
addpath(genpath('./'));
map = load_map('map1.txt', 0.1, 2.0, 0.25);
% num robots and goals
var.nbots = 5;

% num dimensions
var.n = 3;

% radius
var.R = 0.15; % 0.086 m
% boundary x axis and y axis
var.bound = [10,10,40]*1.7;

% max vel
var.vmax = 1;

% st time
var.t0 = 0;

% neighb dist
var.h = 3.7;

%% Assign the goals
% %% Plan path
% disp('Planning ...');
% map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);

[start,goal] = generateStartGoal_dcapt(var, true);
start(:,3) = start(:,3)/4;
goal(:,3) = goal(:,3)/4;

% start = [0,0,0;
%         0,10,0;
%         10,0,0;
%         10,10,0];%[start(:,1),start(:,2),start(:,3)];
% goal = [10,10,10;
%          10,0,10;
%          0,10,10;
%          0,0,10];%[goal(:,1),goal(:,2),goal(:,3)];

var.goal = goal;
var.start = start;
%% init script
var.t0=0; 
var.tf = computeTf_dcapt(start, goal, var.vmax);
times = [var.t0;var.tf];
% var.f = [1,2,3,4];
var.f = randperm(var.nbots);
%% Run trajectory
for qn = 1:var.nbots 
    startC{qn} = start(qn,:)';
    goalC{qn} = goal(qn,:)';
end

trajectory = test_trajectory_dcapt(var, startC, goalC, map, true); % with visualization







