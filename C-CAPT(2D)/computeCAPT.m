function state = computeCAPT(var,t)
%% Compute state for the instant t
% state - N x n matrix ( num-dim by num-states)
% time parameterization
beta = var.alpha_0 + var.alpha_1*t;
state = (1-beta)*var.X_0 + beta*(var.Phi*var.G + ...
                (eye(var.nbots*var.n)-var.Phi*var.Phi')*var.X_0);           
state = reshape(state,[var.n,var.nbots]);
state = state';
end