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
var.nbots = 5;
var.ngoals = 10;
%  var.gap = 5;


% num dimensions
var.n = 3;

% radius
var.R = 0.15; % 0.086 m

% boundary x axis and y axis
var.bound = 10;

% max vel
var.vmax = 0.3;

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

%% Assign Robots to respective goal from the Hungarian output 

% the radius of the robot
scale = 20;
goal = [goal(:,1),goal(:,2),goal(:,3)*4];
start = [start(:,1),start(:,2),start(:,3)*4];
currentgoals = goal;
currentstate = start;
flag = 0;
var.tinit = 0;

%% If there are more goals than goals, run the algorithm iteratively
        % first assigning all the robots to some goals. Remove those goals and
        % run again.
        while size(currentgoals,1)>var.nbots
            % run simulation
            
            [r,g] = find(var.phi==1);
            robot2goals = zeros(var.nbots,var.n);
            robot2goals(r,:) = goal(g,:);
            if var.ngoals<var.nbots
                [notassigned,~] = find(sum(var.phi,2)==0);
                robot2goals(notassigned,:) = start(notassigned,:);
            end
            newgoals = robot2goals;
            
            times = [var.t0;var.tf];
            iter = 1;
            for qn = 1:var.nbots
                path{qn} = [start(qn,:);newgoals(qn,:)];
                coeff{iter,qn} = computeTrajCoeff([], path{qn}, times); 
            end
            iter = iter + 1;
                                  
            % remove the visited goals
            visitedgoals = sum(var.phi,1);
            currentgoals = currentgoals((~visitedgoals),:);
           
            var.t0 = var.tf;
            var.tf1 = var.tf;
            newstart = newgoals;
            var = init(newstart,currentgoals,var);
            flag = 1;
            
        end

        
        
%% Run the last time (if M>N) or for the first time        
if flag==1
    [r,g] = find(var.phi==1);
    robot2goals = zeros(var.nbots,var.n);
    robot2goals(r,:) = currentgoals(g,:);
    if size(currentgoals,1)<var.nbots
        [notassigned,~] = find(sum(var.phi,2)==0);
        robot2goals(notassigned,:) = newstart(notassigned,:);
    end
    newgoals = robot2goals;
    times = [var.t0;var.tf];
    for qn = 1:var.nbots
        path{qn} = [newstart(qn,:);newgoals(qn,:)];
        coeff{iter,qn} = computeTrajCoeff([], path{qn}, times);        
    end
end


trajectory_generatorGoalsmorethanBots([],[],var,coeff,path);

%% Run trajectory
for qn = 1:var.nbots 
    startC(qn) = {start(qn,:)};
    goalC(qn) = {newgoals(qn,:)};
end

trajectory = test_trajectoryGoalsmorethanBots(startC, goalC, map, path, true,var); % with visualization










