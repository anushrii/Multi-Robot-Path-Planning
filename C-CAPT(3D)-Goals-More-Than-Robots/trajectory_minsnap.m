function desired_state = trajectory_minsnap(timeViaPoints,t,ax,ay )

for i=1:length(timeViaPoints)-1

        if t>=timeViaPoints(i) && t< timeViaPoints(i+1)
           
          x = ax(i,1) + ax(i,2)*t + ax(i,3)*t^2 + ax(i,4)*t^3 + ax(i,5)*t^4 + ax(i,6)*t^5 + ax(i,7)*t^6 + ax(i,8)*t^7;	
          y = ay(i,1) + ay(i,2)*t + ay(i,3)*t^2 + ay(i,4)*t^3 + ay(i,5)*t^4 + ay(i,6)*t^5 + ay(i,7)*t^6 + ay(i,8)*t^7;	
%           z = az(i,1) + az(i,2)*t + az(i,3)*t^2 + az(i,4)*t^3 + az(i,5)*t^4 + az(i,6)*t^5;
          
          dx = ax(i,2) + 2*ax(i,3)*t + 3*ax(i,4)*t^2 + 4*ax(i,5)*t^3 + 5*ax(i,6)*t^4 + 6*ax(i,7)*t^5 + 7*ax(i,8)*t^6;	
          dy = ay(i,2) + 2*ay(i,3)*t + 3*ay(i,4)*t^2 + 4*ay(i,5)*t^3 + 5*ay(i,6)*t^4 + 6*ay(i,7)*t^5 + 7*ay(i,8)*t^6;	
%           dz = az(i,2) + 2*az(i,3)*t + 3*az(i,4)*t^2 + 4*az(i,5)*t^3 + 5*az(i,6)*t^4;
          
          d2x = 2*ax(i,3) + 6*ax(i,4)*t + 4*3*ax(i,5)*t^2 + 5*4*ax(i,6)*t^3 + 5*6*ax(i,7)*t^4 + 5*7*ax(i,8)*t^5;	
          d2y = 2*ay(i,3) + 6*ay(i,4)*t + 4*3*ay(i,5)*t^2 + 5*4*ay(i,6)*t^3 + 5*6*ay(i,7)*t^4 + 5*7*ay(i,8)*t^5;	
%           d2z = 2*az(i,3) + 6*az(i,4)*t + 4*3*az(i,5)*t^2 + 5*4*az(i,6)*t^3;
          
          pos = [x; y];% z];
          vel = [dx; dy];% dz];
          acc = [d2x; d2y];% d2z];
%          
%           x = ax(i,1) + ax(i,2)*t + ax(i,3)*t^2 + ax(i,4)*t^3;	
%           y = ay(i,1) + ay(i,2)*t + ay(i,3)*t^2 + ay(i,4)*t^3;	
% %           z = az(i,1) + az(i,2)*t + az(i,3)*t^2 + az(i,4)*t^3;
%           
%           dx = ax(i,2) + 2*ax(i,3)*t + 3*ax(i,4)*t^2;	
%           dy = ay(i,2) + 2*ay(i,3)*t + 3*ay(i,4)*t^2;	
% %           dz = az(i,2) + 2*az(i,3)*t + 3*az(i,4)*t^2;
%           
%           d2x = 2*ax(i,3) + 6*ax(i,4)*t;	
%           d2y = 2*ay(i,3) + 6*ay(i,4)*t;	
% %           d2z = 2*az(i,3) + 6*az(i,4)*t;
%           
%           pos = [x; y]; %z];desired_state
%           vel = [dx; dy]; %dz];
%           acc = [d2x; d2y];% d2z];
         
        end
%================================================================================================    
          if(t>=timeViaPoints(end))
          
          x = ax(i,1) + ax(i,2)*t + ax(i,3)*t^2 + ax(i,4)*t^3 + ax(i,5)*t^4 + ax(i,6)*t^5 + ax(i,7)*t^6 + ax(i,8)*t^7;	
          y = ay(i,1) + ay(i,2)*t + ay(i,3)*t^2 + ay(i,4)*t^3 + ay(i,5)*t^4 + ay(i,6)*t^5 + ay(i,7)*t^6 + ay(i,8)*t^7;	
          
          pos = [x; y];
          vel = [0; 0];
          acc = [0; 0 ];
              
          end

end    
%===============================================================================================
 
%===============================================================================================
yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end