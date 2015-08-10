function xyz = subs2xyz(map, pts)
%% Convert node indices 1:nx, 1:ny, 1:nz to x,y,z positions
    boundary = map.boundary;
    xy_res = map.xy_res;
    z_res = map.z_res;
    xyz = zeros(size(pts)); 
    xyz(:,1) = (pts(:,1)-1)*xy_res + boundary(1);
    xyz(:,2) = (pts(:,2)-1)*xy_res + boundary(2);
    xyz(:,3) = (pts(:,3)-1)*z_res + boundary(3);
end