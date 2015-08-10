function [subs] = xyz2sub(map, pts)
%% convert xyz points in 3d to subs 
boundary = map.boundary;    
i = round( (pts(:,1)-boundary(1) )/ map.xy_res ) +1 ; % xmax-xmin/xres
j = round( (pts(:,2)-boundary(2) )/ map.xy_res ) +1; % ymax-ymin/yres
k = round( (pts(:,3)-boundary(3) )/ map.z_res ) +1; % zmax-zmin/zres

subs = [i,j,k];

end