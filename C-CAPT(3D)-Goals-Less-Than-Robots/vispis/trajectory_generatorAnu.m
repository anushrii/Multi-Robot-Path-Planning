function [ desired_state ] = trajectory_generatorAnu(t, qn, var, coeff,path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;
%tmax clock1
persistent allcoeff variables path0
if nargin == 5
% map0   = map;
allcoeff = coeff;
variables = var;
path0 =path;
% clock1 = clock;
%% code here the reduced set of nodes for smoothening 

% ==============================================================================================

% if the nodes need to be reduced/ remain the same as the path:
    
%         path0  = path; 
%         [viaPoints, timeViaPoints] = getNodes(map,path0);
% 
%     %===============================================================================================
%         ax=getCoeffsMinsnap(viaPoints,timeViaPoints,1);
%         ay=getCoeffsMinsnap(viaPoints,timeViaPoints,2);
%         az=getCoeffsMinsnap(viaPoints,timeViaPoints,3);
%         coeff = [ax ay az];
%         desired_state = [];
%     


%===============================================================================================
  
%===============================================================================================
else
    
    currentcoeff = allcoeff{qn};
    ax = currentcoeff(:,1);
    ay = currentcoeff(:,2);
    az = currentcoeff(:,3);

    if t>=variables.t0 && t<variables.tf



        x = ax(1) + ax(2)*t + ax(3)*t^2 + ax(4)*t^3 + ax(5)*t^4 + ax(6)*t^5 + ax(7)*t^6 + ax(8)*t^7;
        y = ay(1) + ay(2)*t + ay(3)*t^2 + ay(4)*t^3 + ay(5)*t^4 + ay(6)*t^5 + ay(7)*t^6 + ay(8)*t^7;
        z = az(1) + az(2)*t + az(3)*t^2 + az(4)*t^3 + az(5)*t^4 + az(6)*t^5 + az(7)*t^6 + az(8)*t^7;

        dx = ax(2) + 2*ax(3)*t + 3*ax(4)*t^2 + 4*ax(5)*t^3 + 5*ax(6)*t^4 + 6*ax(7)*t^5 + 7*ax(8)*t^6;
        dy = ay(2) + 2*ay(3)*t + 3*ay(4)*t^2 + 4*ay(5)*t^3 + 5*ay(6)*t^4 + 6*ay(7)*t^5 + 7*ay(8)*t^6;
        dz = az(2) + 2*az(3)*t + 3*az(4)*t^2 + 4*az(5)*t^3 + 5*az(6)*t^4 + 6*az(7)*t^5 + 7*az(8)*t^6;

        d2x = 2*ax(3) + 6*ax(4)*t + 4*3*ax(5)*t^2 + 5*4*ax(6)*t^3 + 5*6*ax(7)*t^4 + 5*7*ax(8)*t^5;
        d2y = 2*ay(3) + 6*ay(4)*t + 4*3*ay(5)*t^2 + 5*4*ay(6)*t^3 + 5*6*ay(7)*t^4 + 5*7*ay(8)*t^5;
        d2z = 2*az(3) + 6*az(4)*t + 4*3*az(5)*t^2 + 5*4*az(6)*t^3 + 5*6*az(7)*t^4 + 5*7*az(8)*t^5;

        pos = [x; y;z];
        vel = [dx; dy; dz];
        acc = [d2x; d2y; d2z];    


    end
%================================================================================================
    if t>=variables.tf

         t = variables.tf;
        x = ax(1) + ax(2)*t + ax(3)*t^2 + ax(4)*t^3 + ax(5)*t^4 + ax(6)*t^5 + ax(7)*t^6 + ax(8)*t^7;
        y = ay(1) + ay(2)*t + ay(3)*t^2 + ay(4)*t^3 + ay(5)*t^4 + ay(6)*t^5 + ay(7)*t^6 + ay(8)*t^7;
        z = az(1) + az(2)*t + az(3)*t^2 + az(4)*t^3 + az(5)*t^4 + az(6)*t^5 + az(7)*t^6 + az(8)*t^7;
  

        pos = [path0{qn}(end,1); path0{qn}(end,2); path0{qn}(end,3)]

%         pos = [x; y; z]
        vel = [0; 0; 0];
        acc = [0; 0; 0];

    end

%         end
%===============================================================================================



yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
end

