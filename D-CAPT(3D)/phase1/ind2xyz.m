function [pts] = ind2xyz(map,lininds)
%% convert linear node indices to xyz
    [i,j,k] = ind2sub(map.dim, lininds);
    subs = [i,j,k];
    pts = subs2xyz(map, subs);
end