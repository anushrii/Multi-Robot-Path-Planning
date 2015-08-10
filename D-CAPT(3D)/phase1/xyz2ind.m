function [linind] = xyz2ind(map,pts)
%% convert xyz points to linear node indices
    [subs] = xyz2sub(map,pts);
    linind = sub2ind(map.dim,subs(:,1),subs(:,2),subs(:,3));
end