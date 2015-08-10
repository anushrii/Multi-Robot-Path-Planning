function var = init(start,goal,var)
%% Generate distance matrix from all starts to all goals
% start init positions of robot (N*n)
% goal - pos of robot (M*n)
% var has the properties nbots, ngoals, vmax
% var is added properties alpha_0 alpha_1 tf Phi G X_0 phi

%% extract the parameters
nbots = size(start,1);
ngoals = size(goal,1);
n = var.n;
vmax = var.vmax;
t0 = var.t0;

%% Compute the distance squared and then get optimal assignment
distsq = (pdist2(start,goal)).^2; % N x M
[goal_idx,~] = munkres(distsq);

%% Build the phi matrix from the assignment
phi = zeros(nbots,ngoals);
for robot_idx=1:nbots
    if goal_idx(robot_idx)==0
        continue;
    else
        phi(robot_idx,goal_idx(robot_idx)) = 1;
    end
end

%% Compute the Nn x Mn n dimensional assignment matrix Phi, Extract the
Phi = kron(phi,eye(n));
temp1 = start';
temp2 = goal';

X_0 = temp1(:); % Nn
G = temp2(:); % Mn

%% compute the init parameters

% pick out the distances which are being used and extract
% max dist and then divide by vmax
max_dist_sq = max(distsq(phi(:)>0));
tf = sqrt(max_dist_sq)/vmax;

% time parameterization Beta(t) = alpha_0 + alpha_1*t
alpha_0 = -t0/(tf-t0);
alpha_1 = 1/(tf-t0);

%% Assignment of results
var.alpha_0 = alpha_0;
var.alpha_1 = alpha_1;
var.tf = tf;
var.Phi = Phi;
var.G = G;
var.X_0 = X_0;
var.phi = phi;
end