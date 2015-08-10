function [start,goal] = generateStartGoal_dcapt(var, is_N_equal_M)
%% Generates start and goal positions for the bots
% var is the ds with all the parameters
% start - starting points of dimensions (num_robots x ndim) (NxN)
% goal - goal points of dimensions (num_robots x ndim) (NxN)

ndim = var.n;

nbots = var.nbots;

% in case of dcapt num goals same as num bots
if nargin==1 || ~is_N_equal_M
    ngoals = var.ngoals;
else
    ngoals = nbots;
end

bound = var.bound;
start=[];
goal =[];
if length(bound)==1
    bound = ones(1,ndim)*nbots;
end
while true
    for i=1:ndim
        start = [start,randi(bound(i),nbots,1)];
        goal = [goal,randi(bound(i),ngoals,1)];
    end
        allPoints = [start;goal];
        % repeat until num of all unique points
        if(numel(allPoints)==numel(unique(allPoints,'rows')))
            break;
        end
end
end

