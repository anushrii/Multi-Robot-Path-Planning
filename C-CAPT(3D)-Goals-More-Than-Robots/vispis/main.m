%% Main with simulation

%% Clear the previous
clear 
close all
clc
%% Parameters
disp('Planning ...');
addpath(genpath('./'));
map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);
% num robots and goals
var.nbots = 10;
var.ngoals = 10;

% num dimensions
var.n = 3;

% radius
var.R = 0.15; % 0.086 m

% boundary x axis and y axis
var.bound = 10;

% max vel
var.vmax = 1.5;

% st time
var.t0 = 0;

%% Assign the goals
% %% Plan path
% disp('Planning ...');
% map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);

[start,goal] = generateStart_Goal(var);
goal = [goal(:,1),goal(:,2),goal(:,3)/4];
start = [start(:,1),start(:,2),start(:,3)/4];

% compute the assignment, the time parameterization and
% the coffiecients to compute the state at every instant
% first time to get tf or in case to get init N>=M
var = init(start,goal,var);



% the radius of the robot
scale = 20;
goal = [goal(:,1),goal(:,2),goal(:,3)*4];
start = [start(:,1),start(:,2),start(:,3)*4];
currentgoals = goal;
currentstate = start;


%% Additional init script
% init_script;
var.t0 = 0;
var.tf = 10;
times = [var.t0;var.tf];
for qn = 1:var.nbots    
    path{qn} = [start(qn,:);goal(qn,:)]; 
    coeff{qn} = computeTrajCoeff([], path{qn}, times); 
    
end

trajectory_generatorAnu([],[],var,coeff,path);

%% Run trajectory
for qn = 1:var.nbots 
    startC(qn) = {start(qn,:)};
    goalC(qn) = {goal(qn,:)};
end

trajectory = test_trajectory(startC, goalC, map, path, true); % with visualization










