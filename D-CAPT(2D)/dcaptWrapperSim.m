function dcaptWrapperSim(var)
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
U_t = C_prev;
X_c = X0;
t0 = var.t0;
tf = var.tf;

tprev = t0;

t_step = var.t_step;
t_disc = (t0+t_step):t_step:(tf-t_step);

f = var.f;

%%%%%
h_fig = figure('Name', 'Environment');
%========================================== Initial settings and condition========================================%
h_3d = gca;
drawnow;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);
set(gcf,'Renderer','OpenGL')

fprintf('Setting initial conditions...\n')
% Maximum time that the quadrotor is allowed to fly
time_tol = 500;          % maximum simulation time
starttime = 0;          % start of simulation in seconds
tstep     = 0.01;       % this determines the time step at which the solution is given
cstep     = 0.05;       % image capture time interval
nstep     = cstep/tstep;
time      = starttime;  % current time
max_iter  = time_tol / cstep;      % max iteration

% %     for qn = 1:nquad
% %         % Get start and stop position
% %         x0{qn}    = init_state(start{qn}, 0);
% %         xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
% %         ttraj{qn} = zeros(max_iter*nstep, 1);
% %     end

% Maximum position error of the quadrotor at goal
pos_tol  = 0.05; % m
% Maximum speed of the quadrotor at goal
vel_tol  = 0.05; % m/s

x = x0;        % state


%%%%
fprintf('Simulation Running....\n')
for iter = 1:max_iter
    timeint = time:tstep:time+cstep;
    tic; % t = tic???
    % for it = 1:numel(t_disc)
    tc = t_disc(it);
    isFC = true(N,1);
    
    %     X_c = computeState(X_c, G(f,:), tprev, tc, tf);
  
    %     var.quadPlot.update(X_c);
    
    %%%==================================================================================================%%%
    
    times = [tc;tf];
    for qn = 1:var.nbots
        path{qn} = [X_c(qn,:);G(f(qn),:)];
        coeff{qn} = computeTrajCoeff([], path{qn}, times);
        
    end
       trajectory_generatorAnu([],[],var,coeff,path);
       X_c  = trajectory_generatorAnu(t, qn, var, coeff,path);
 
    
    %%% Run trajectory
    for qn = 1:var.nbots
        startC(qn) = {X_c(qn,:)};
        goalC(qn) = {G(f(qn),:)};
    end
    
    if iter == 1
        vis = true;
    else
        vis = false;
    end
    
    trajectory = plotSimulation(startC, goalC, var.map, path, vis, time,quadcolors); % with visualization  %% vis == True
    
    %%%==================================================================================================%%%
    
    C_t = computeNeighb(X_c, var.h);
    if(any(C_t))
        lemon=1;
    end
    U_t = U_t | (C_t & (~C_prev));
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
                    
                    times = [tc;tf];
                    for qn = 1:var.nbots
                        path{qn} = [X_c(ri,:); G(f(ri),:)];
                        coeff{qn} = computeTrajCoeff([], path{qn}, times);
                        
                    end
                    
                    trajectory_generatorAnu([],[],var,coeff,path);
                     X_c(ri,:)  = trajectory_generatorAnu(t, qn, var, coeff,path);
                     X_c(rj,:)  = trajectory_generatorAnu(t, qn, var, coeff,path);
                    
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
    
    
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    time = time + cstep; % Update simulation time
    t = toc;
    
    % Pause to make real-time
    if (t < cstep)
        pause(cstep - t);
    end
    
    % Check termination criteria
    terminate_cond = terminate_check(x, time, stop, pos_tol, vel_tol, time_tol);
    if terminate_cond
        break
    end
end

%% ************************* POST PROCESSING *************************
fprintf('Simulation Finished....\n')
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

% Plot the saved position and velocity of each robot
if vis
    for qn = 1:nquad
        % Truncate saved variables
        QP{qn}.TruncateHist();
        % Plot position for each quad
        h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
        plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
        plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
        % Plot velocity for each quad
        h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
        plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
        plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'des');
    end
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

function Xnext = computeState(Xc, Gc, tc, t, tf)
%% Given the current state and current goals and the
% current time, the time to estimate the next state
% and the end time compute Xnext
beta = (t-tc)/(tf-tc);
Xnext = (1-beta)*Xc + beta*Gc;
end

function toSwap = isSwap(x_i, g_i, x_j, g_j)
%% checks if u'*w <0; where u = xj_c - xj_c
% checks if u'*w <0; where w = xj_tf - xj_tf

U = x_j - x_i; % 1 by n
W = g_j - g_i; % 1 by n

toSwap = U*W' < 0; % 1 by n x n by 1

end