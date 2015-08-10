function [F, M, trpy, drpy] = controller_dcapt(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Set the gains
% kp_trans =[6.5  6.5  7]';
% kd_trans =[5  5  2]';
% 
% kp_rot = [4 4 3]';
% kd_rot = [.1 .1 0.15 ]';
% kp_trans =[14+10  14+10  10.5+25]';
% kd_trans =[7.5  4.5  6]';
% 
% kp_rot = [6 1.5 5]';
% kd_rot = [.1 .05 0.5 ]';

kp0_trans = [ 3 3 5]'; 
kd0_trans = [0.05 0.05 0.05]';

kpf_trans = [14+10  14+10  10.5+25]'; 
kdf_trans = [7.5  4.5  6]';

kp0_rot = [2 1.5 2]';
kd0_rot = [0.5 .5 0.5 ]';

kpf_rot = [6 1.5 5]';
kdf_rot = [.1 .05 0.5 ]';

% if t<0.5
%     kp_trans =[10  10  15]';
%     kd_trans =[5  5  10]';
% else
%     kp_trans =[25  25  30]';
%     kd_trans =[9  9  20]';
% end
% kp_rot = [10 10 10]';
% kd_rot = [0.1 .1 2 ]';
t0=0;
tf = 1;

if t<tf
kp_trans = computeLin(kp0_trans, kpf_trans,t0, tf, t); 
kd_trans = computeLin(kd0_trans, kdf_trans,t0, tf, t); 

kp_rot = computeLin(kp0_rot, kpf_rot,t0, tf, t); 
kd_rot = computeLin(kd0_rot, kdf_rot,t0, tf, t); 
else
kp_trans =  kpf_trans; 
kd_trans = kdf_trans;

kp_rot = kpf_rot;
kd_rot = kdf_rot;
end

% extract the state variables
pos = qd{qn}.pos;
vel = qd{qn}.vel;
euler = qd{qn}.euler; % [phi, theta, psi]
omega = qd{qn}.omega;

pos_des = qd{qn}.pos_des;
vel_des = qd{qn}.vel_des;
acc_des = qd{qn}.acc_des;
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;

% constants 
m =  params.mass;
I = params.I;
invI = params.invI;
g = params.grav;
L = params.arm_length;
maxangle = 0.65; %0.6 something
maxF = params.maxF;%4.3
minF = params.minF;%0.0863


% rcommand
r_cmd = acc_des +  ( kd_trans.*(vel_des - vel) ) +  ( kp_trans.*( pos_des - pos ) );

% Thurst u1
F    =  min(max(m*( g +  r_cmd(3)), minF), maxF);

% Desired roll, pitch and yaw
phi_des = (1.0/g) * ( ( r_cmd(1)*sin(yaw_des) ) - ( r_cmd(2)*cos(yaw_des) ) ) ;
phi_des = sign(phi_des) * min(abs(phi_des), maxangle);

theta_des = (1.0/g) * ( ( r_cmd(1)*cos(yaw_des) ) + ( r_cmd(2)*cos(yaw_des) ) ) ;
theta_des = sign(theta_des) * min(abs(theta_des), maxangle);

psi_des = yaw_des;

euler_des = [phi_des; theta_des; psi_des] ; 
omega_des = [0; 0; yawdot_des]; % [p q r]

% Moment
M = kp_rot.*(euler_des - euler) + kd_rot.*(omega_des-omega);
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];


end

function [q] = computeLin(q0, qf,t0, tf, t)
    q = q0 +( ( (qf-q0)/(tf-t0) ) * (t-t0)  ) ;
end