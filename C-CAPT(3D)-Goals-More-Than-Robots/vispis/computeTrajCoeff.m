% function [coeff] = computeTrajCoeff(map, path, times)
function [coeff] = computeTrajCoeff(~, path, times)
% path0  = path;
% [viaPoints, ~] = getNodes(map,path0);
viaPoints = path;
ax=getCoeffsMinsnap(viaPoints,times,1);
ay=getCoeffsMinsnap(viaPoints,times,2);
az=getCoeffsMinsnap(viaPoints,times,3);
coeff = [ax ay az];
end