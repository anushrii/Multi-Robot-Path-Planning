%% Main with simulation

%% Clear the previous
clear 
close all
clc
%% Parameters
disp('Planning ...');
addpath(genpath('./'));
% addpath('C:\Users\PC\Desktop\MS\Spring 2015\Advanced Robotics\Project-3\New folder\multi-robot-path-planning-master');
map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);
% num robots and goals
var.nbots = 10;
var.ngoals = 5;


% num dimensions
var.n = 3;

% radius
var.R = 0.15; % 0.086 m

% boundary x axis and y axis
var.bound = 10;

% max vel
var.vmax = 0.7;

% st time
var.t0 = 0;

%% Assign the goals
% %% Plan path
% disp('Planning ...');
% map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);

[start,goal] = generateStart_Goal(var);
% start = [1 1 1;3 3 3;4 4 4;0 2 3; 1 5 8;];
% goal = [6 2 8;6 6 6;8 8 8;4 5 1; 2 2 2];
% start = [1 1 1];
% goal = [6 6 6];

goal = [goal(:,1),goal(:,2),goal(:,3)/4];
start = [start(:,1),start(:,2),start(:,3)/4];

% compute the assignment, the time parameterization and
% the coffiecients to compute the state at every instant
% first time to get tf or in case to get init N>=M
var = init(start,goal,var);
[r,g] = find(var.phi==1);
robot2goals = zeros(var.nbots,var.n);
robot2goals(r,:) = goal(g,:);
if var.ngoals<var.nbots
    [notassigned,~] = find(sum(var.phi,2)==0);
    robot2goals(notassigned,:) = start(notassigned,:);
end
newgoals = robot2goals;

% the radius of the robot
scale = 20;
newgoals = [newgoals(:,1),newgoals(:,2),newgoals(:,3)*4];
start = [start(:,1),start(:,2),start(:,3)*4];
currentgoals = newgoals;
currentstate = start;



%% Additional init script
% init_script;
% var.t0 = 0;
% var.tf = 10;
times = [var.t0;var.tf];
for qn = 1:var.nbots    
    path{qn} = [start(qn,:);newgoals(qn,:)]; 
    coeff{qn} = computeTrajCoeff([], path{qn}, times); 
    
end

trajectory_generatorGoalslessthanBots([],[],var,coeff,path);

%% Run trajectory
for qn = 1:var.nbots 
    startC(qn) = {start(qn,:)};
    goalC(qn) = {newgoals(qn,:)};
end

trajectory = test_trajectoryGoalslessthanBots(startC, goalC, map, path, true); % with visualization










