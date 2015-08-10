function dcaptWrapper(var)
%% The function which performs the dcapt computation and simulation
% var is a structure containing start goal f(assignment of goal) nbots
% bound vmax bound

%% Assign params
X0 = var.start;
G = var.goal;

N = var.nbots;

% compute final time
var.tf = computeTf(var.start, var.goal(var.f,:), var.vmax);
% neig
C_prev = computeNeighb(var.start, var.h);
U_prev = C_prev;
X_c = X0;
t0 = var.t0;
tf = var.tf;

tprev = t0;

t_step = var.t_step;
t_disc = (t0+t_step):t_step:(tf-t_step);

f = var.f;

for it = 1:numel(t_disc)
    tc = t_disc(it);
    isFC = true(N,1);

    X_c = computeState(X_c, G(f,:), tprev, tc, tf);
    var.quadPlot.update(X_c);


    
    C_t = computeNeighb(X_c, var.h);
    U_t = U_prev | (C_t & (~C_prev));
    U_t = U_t & C_t;
    
    while(any(isFC))
        
        %robots to check
        rtc = find(isFC);
        
        for i = 1:numel(rtc)

            ri = rtc(i);
            isFC(ri) = false;
            
            % neighb
            n_ind = find(U_t(ri,:));
            
            for j = 1:numel(n_ind)
                % neighb
                rj = n_ind(j);
                
                % any has to be swapped
                if isSwap(X_c(ri,:), G(f(ri),:), X_c(rj,:), G(f(rj),:))
                    
                    % swap fi fj
                    tmp = f(ri);
                    f(ri) = f(rj);
                    f(rj) = tmp';
                    
                    % new neighb
                    U_t(ri,:) = C_t(ri,:);
                    U_t(ri,rj) = false;
                    U_t(rj,:) = C_t(rj,:);
                    U_t(rj,ri) = false;
                    
                    % recompute trajs
                    X_c(ri,:) = computeState(X_c(ri,:),G(f(ri),:), tprev, tc, tf);                   
                    X_c(rj,:) = computeState(X_c(rj,:),G(f(rj),:), tprev, tc, tf);
                    
                    isFC(ri) = true;
                    isFC(rj) = true;
                    
                end
                U_t(ri,rj) = false;
            end
        end
    end
    
    %plotSim(X_c, var, var.start, var.goal, var.scale)
    tprev = tc;
    C_prev = C_t;
    U_prev = U_t;
end
    var.quadPlot.update(G(f,:));
end

function tf = computeTf(start, goal, vmax)
%% computes the max time that would take with the current
% assignment
% start N by ndim
% goal N by ndim

diff = (start-goal);
dist = sqrt(sum(diff.^2,2));

tf = max(dist)/vmax;

end

function C = computeNeighb(X, h )
%% X = N by ndim position
% C = N by N sparse matrix 1 for points which are less than h dist
% btw them
N = size(X,1);
C = pdist2(X,X)<=h;
% make diag zeros
C(logical(eye(N))) = 0;
end

function Xnext = computeState(Xprev, Gc, tprev, tc, tf)
%% Given the current state and current goals and the
% current time, the time to estimate the next state
% and the end time compute Xnext
beta = (tc-tprev)/(tf-tprev);
Xnext = (1-beta)*Xprev + beta*Gc;
end

function toSwap = isSwap(x_i, g_i, x_j, g_j)
%% checks if u'*w <0; where u = xj_c - xj_c
% checks if u'*w <0; where w = xj_tf - xj_tf

U = x_j - x_i; % 1 by n
W = g_j - g_i; % 1 by n

toSwap = U*W' < 0; % 1 by n x n by 1 

end