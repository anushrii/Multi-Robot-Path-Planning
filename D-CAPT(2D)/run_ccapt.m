%% Clear the previous
clear 
close all

%% Parameters

% num robots and goals
var.nbots = 4;
var.ngoals = 4;

% num dimensions
var.n = 2;

% radius
var.R = 0.3;

% boundary x axis and y axis
var.bound = 100;

% max vel
var.vmax = 30;

% st time
var.t0 = 0;

%% Assign the goals

[start,goal] = generateStartGoal(var);

% compute the assignment, the time parameterization and
% the coffiecients to compute the state at every instant
% first time to get tf or in case to get init N>=M
var = init(start,goal,var);

t_step = 0.01;

% the radius of the robot
scale = 20;
currentgoals = goal;
currentstate = start;

%% If there are more goals than goals, run the algorithm iteratively
% first assigning all the robots to some goals. Remove those goals and
% run again.

while size(currentgoals,1)>var.nbots
    % run simulation
    t_disc = [var.t0:t_step:var.t0+var.tf,var.tf];
    for i = 1:numel(t_disc)
        
        t = t_disc(i);
        state = computeCAPT(var,t);
        
        % Plot the current state
        plotSim(state, var, start, goal, scale);
        pause(0.0001)
    end
    
    % remove the visited goals
    visitedgoals = sum(var.phi,1);
    currentgoals = currentgoals((~visitedgoals),:);
    var.t0 = 0;
    currentstate = state;
    var = init(currentstate,currentgoals,var);
end

%% Run the last time (if M>N) or for the first time

for t=0:t_step:var.tf
    state = computeCAPT(var,t);

    % Plot the current state
    plotSim(state, var, start, goal, scale);
    
    
    pause(0.0001)
end