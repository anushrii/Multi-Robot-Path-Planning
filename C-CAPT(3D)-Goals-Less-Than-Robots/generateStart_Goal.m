function [start,goal] = generateStart_Goal(var)
%% Generates start and goal positions for the bots
% var is the ds with all the parameters
% start - starting points of dimensions (num_robots x ndim) (NxN)
% goal - goal points of dimensions (num_robots x ndim) (NxN)

ndim = var.n;
ngoals = var.ngoals;
nbots = var.nbots;
bound = var.bound;

while true
    start = randi(bound,nbots,ndim);
    goal = randi(bound,ngoals,ndim);
    allPoints = [start;goal];
    % repeat until num of all unique points
    if(numel(allPoints)==numel(unique(allPoints,'rows')))
        break;
    end
end
end

