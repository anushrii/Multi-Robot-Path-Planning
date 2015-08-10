% Add additional initialization if you want to.
% You can use this space to pre-compute the trajectory to avoid
% repeatedly computing the same trajectory in every call of the
% "trajectory_generator" function

% Generate trajectory
disp('Generating Trajectory ...');
waypoints = cell2mat(path(1));
xyres = map(3,4);
zres = map(4,4);
modifiedpath = waypoints;


iter2newpath = [];
iter2newpath(1,:) = waypoints(1,:);
i = 1;

while sum(iter2newpath(end,:) ~= waypoints(end,:))>0
    j = 0;
    C = 1;
    while C==1
        newpoint = waypoints(end-j,:);
        currentpoint = iter2newpath(i,:);
        checkpoints = mygrid(currentpoint,newpoint,xyres,zres);        
        C = collide(map,checkpoints);
        C = max(C);
        j = j + 1;
    end
    iter2newpath(end+1,:) = newpoint;
    i = i + 1;
end

for i = 1:size(iter2newpath,1)-1
    distbtwpts(i,1) =  pdist2(iter2newpath(i,:),iter2newpath(i+1,:));
end
totaldist = sum(distbtwpts); 
tmin = 0;
velmax = 1.4;
% totaltime = totaldist/velmax;
equations = [];
for i = 1:size(iter2newpath,1)-1
    ratio = distbtwpts(i)/totaldist;
    tmax = tmin + 1.0*sqrt(distbtwpts(i));
    currentstart = iter2newpath(i,:);
    currentstop = iter2newpath(i+1,:);
    currentequation = CalculateEquations(currentstart,currentstop,tmin,tmax);
    equations = [equations;currentequation];
    tmin = tmax;
end

save('allequations.mat','equations');
path = iter2newpath;
trajectory_generator([], [], map, path);