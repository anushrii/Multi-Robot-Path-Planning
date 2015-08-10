function [F, M, trpy, drpy] = controllerAnu(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

phi = qd{qn}.euler(1);
theta = qd{qn}.euler(2);
psi = qd{qn}.euler(3);

maxangle =params.maxangle;
p = qd{qn}.omega(1);
q = qd{qn}.omega(2);
r = qd{qn}.omega(3);

g = params.grav;
m =params.mass;

Vd = qd{qn}.vel_des;
Va = qd{qn}.vel;
Ad = qd{qn}.acc_des;
Pd = qd{qn}.pos_des;
Pa = qd{qn}.pos;
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;

%===============best================
% Kd_x = 4; 
% Kd_y = 3;
% Kd_z = 4;
% 
% Kp_x = 21;
% Kp_y = 15;
% Kp_z = 20;
% 
% Kp_phi = 4;
% Kp_theta = 2;
% Kp_psi = 10;
% 
% Kd_phi = 0.09;
% Kd_theta = 0.07;
% Kd_psi = 1;
%===============best================
Kd_x = 10; 
Kd_y = 10;
Kd_z = 30;

Kp_x = 4;
Kp_y = 4;
Kp_z = 70;

Kp_phi = 0.6;
Kp_theta = 0.6;
Kp_psi = 0.02;

Kd_phi = 0.06;
Kd_theta = 0.06;
Kd_psi = 0.02;

% Kd_x = 4; 
% Kd_y = 3;
% Kd_z = 4;
% 
% Kp_x = 21;
% Kp_y = 15;
% Kp_z = 20;
% 
% Kp_phi = .4;
% Kp_theta = .2;
% Kp_psi = .10;
% 
% Kd_phi = 0.09;
% Kd_theta = 0.07;
% Kd_psi = .1;




% Kd_x = 1; 
% Kd_y = 0.5;
% Kd_z = 0.9;
% 
% Kp_x = 20;
% Kp_y = 20;
% Kp_z = 29;
% 
% Kp_phi = 20;
% Kp_theta = 20;
% Kp_psi = 60;
%
% Kd_phi = 0.09;
% Kd_theta = 0.07;
% Kd_psi = 1;

r_ax_c = Ad(1) + Kd_x*(Vd(1) - Va(1)) + Kp_x*(Pd(1) - Pa(1));
r_ay_c = Ad(2) + Kd_y*(Vd(2) - Va(2)) + Kp_y*(Pd(2) - Pa(2));
r_az_c = Ad(3) + Kd_z*(Vd(3) - Va(3)) + Kp_z*(Pd(3) - Pa(3));


u1 = m*g + m*r_az_c;
phi_des = 1/g*(r_ax_c*sin(yaw_des) - r_ay_c*cos(yaw_des));
theta_des = 1/g*(r_ax_c*cos(yaw_des) - r_ay_c*sin(yaw_des));

if phi_des>maxangle
    phi_des=maxangle;
end

if theta_des>maxangle
    theta_des=maxangle;
end

p_des = 0;
q_des = 0;
r_des = yawdot_des;

psi_des = yaw_des;


u2 = [Kp_phi*(phi_des - phi) + Kd_phi*(p_des - p) ;
      Kp_theta*(theta_des - theta) + Kd_theta*(q_des - q) ;
      Kp_psi*(psi_des - psi) + Kd_psi*(r_des - r) ];
  

% Desired roll, pitch and yaw
% phi_des = 0;
% theta_des = 0;
% psi_des = 0;

% Thurst
F    = u1;

% Moment
M    = u2; % You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
