function [ desired_state ] = trajectory_generatorGoalslessthanBots(t, qn, var, coeff,path)
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
persistent allcoeff variables
if nargin == 5
 
    allcoeff = coeff;
    variables = var;
        
else
    
    
    if t>=variables.t0 && t<variables.tf
        
        currentcoeff = allcoeff{1,qn};     
        [pos,vel,acc] = computeState(currentcoeff,t);   
        

    %================================================================================================
    else
        currentcoeff = allcoeff{1,qn};
        t = variables.tf;       
        [pos,~,~] = computeState(currentcoeff,t);              
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

