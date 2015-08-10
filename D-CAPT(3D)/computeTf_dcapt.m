
function tf = computeTf(start, goal, vmax)
%% computes the max time that would take with the current
% assignment
% start N by ndim
% goal N by ndim

diff = (start-goal);
dist = sqrt(sum(diff.^2,2));

tf = max(dist)/vmax;

end