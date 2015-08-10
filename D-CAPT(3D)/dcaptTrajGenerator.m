function [desired_state,fret] = dcaptTrajGenerator(var,tc,state, qn)
%% The function which performs the dcapt computation and simulation
% var is a structure containing start goal f(assignment of goal) nbots
% bound vmax bound

%% Assign params
persistent tprev C_prev U_prev f coeff G

N = var.nbots;
tf = var.tf;

if(tc>=tf)
    desired_state.pos = G(f(qn),:)';
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0; 
    fret =f;
    return;
end

if isempty(tprev)
    G = var.goal;
    f = var.f;
    C_prev = computeNeighb(var.start, var.h);
    U_prev = C_prev;
    tprev = var.t0;
    %% TODO set the des
    X_V_0 = extractPosVel(state);
    coeff = computeCoeff(X_V_0, G(f,:), tc, var.tf, true(N,1));
    desired_state.pos = X_V_0(qn,1:3)';
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    fret = f;
    return;
end

%% Actual algorithm
X_V_prev = extractPosVel(state);
X_prev = X_V_prev(:,1:3);
isFC = false(N,1);
isFC(qn) = true;
C_t = computeNeighb(X_prev, var.h);

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
            if isSwap(X_prev(ri,:), G(f(ri),:), X_prev(rj,:), G(f(rj),:))
                
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
                r_inds = [ri;rj]; 
                coeff_ij = computeCoeff(X_V_prev, G(f,:), tprev, tf , r_inds );
                coeff.x([ri;rj],:) = coeff_ij.x;
                coeff.y([ri;rj],:) = coeff_ij.y;
                coeff.z([ri;rj],:) = coeff_ij.z;
                
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

fret=f;
[Xc,Vc,Ac] = computeState(tc, coeff, qn);
desired_state.pos = Xc;
desired_state.vel = Vc;
desired_state.acc = Ac;
desired_state.yaw = 0;
desired_state.yawdot = 0;
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

function [Xnext,Vnext,Anext] = computeState(tc, coeff, qn)
%% Given the current state and current goals and the
% current time, the time to estimate the next state
% and the end time compute Xnext

tpos = [ 1; tc; tc^2;   tc^3;    tc^4;    tc^5;     tc^6;     tc^7];
tvel = [ 0;  1; 2*tc; 3*tc^2;  4*tc^3;  5*tc^4;   6*tc^5;   7*tc^6];
tacc = [ 0;  0;    2;   6*tc; 12*tc^2; 20*tc^3;  30*tc^4;  42*tc^5];
Xnext = [coeff.x(qn,:)*tpos; coeff.y(qn,:)*tpos; coeff.z(qn,:)*tpos];
Vnext = [coeff.x(qn,:)*tvel; coeff.y(qn,:)*tvel; coeff.z(qn,:)*tvel];
Anext = [coeff.x(qn,:)*tacc; coeff.y(qn,:)*tacc; coeff.z(qn,:)*tacc];

end

function toSwap = isSwap(x_i, g_i, x_j, g_j)
%% checks if u'*w <0; where u = xj_c - xj_c
% checks if u'*w <0; where w = xj_tf - xj_tf

U = x_j - x_i; % 1 by n
W = g_j - g_i; % 1 by n

toSwap = U*W' < 0; % 1 by n x n by 1

end

function X = extractPosVel(state)
%% extract pos and vel
    X = cell2mat(state);
    X = X(1:6,:)';
end

function coeff = computeCoeff(X_V, G, tprev, tf , rob_ind)
%% Compute the coefficients of the septic for all the 
% robots if rob_ind is not passed
X_V_int = X_V(rob_ind,:)';
G_int = G(rob_ind,:)';

zrs = zeros(size(X_V_int(1,:)));
x0= X_V_int(1,:);
vx0= X_V_int(4,:);

y0= X_V_int(2,:);
vy0= X_V_int(5,:);

z0= X_V_int(3,:);
vz0= X_V_int(6,:);

xf = G_int(1,:);
yf = G_int(2,:);
zf = G_int(3,:);

% computeSeptic(x0,v0,a0,j0,xf,vf,af,jf, t0,tf)
coeff.x = computeSeptic(x0,vx0,zrs,zrs,xf,zrs,zrs,zrs, tprev,tf)';
coeff.y = computeSeptic(y0,vy0,zrs,zrs,yf,zrs,zrs,zrs, tprev,tf)';
coeff.z = computeSeptic(z0,vz0,zrs,zrs,zf,zrs,zrs,zrs, tprev,tf)';

end