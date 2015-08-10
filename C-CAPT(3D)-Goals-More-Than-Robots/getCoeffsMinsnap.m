function[a]=getCoeffsMinsnap(path,timeViaPoints,xyz)

%% minimum jerk spline init.
% xyz  = 1, coeffs for x dimension
% xyz  = 2, coeffs for y dimension
% xyz  = 3, coeffs for z dimension
% if (xyz == 1)
%     vel = -1;
% elseif (xyz==2)
%     vel = -5;
% end
% number of nodes/viapoints in the trajectory
numNodes = size(path,1);
t=timeViaPoints;

%set of pieces for trajectory through via points
setPieces = numNodes-1;

%set of equations for cubic for each piece = setPieces

a = zeros(setPieces,8);

% do thrice for three dimensions
%% Knowns = invMtrix*coefffs
A = reshape(a',[],1);
knowns = zeros(size(A,1),1);
knowns(1:numNodes-1,1) = path(1:end-1,xyz);
knowns((numNodes):(numNodes + numNodes-2),1) = path(2:end,xyz);
knowns(end) = 0;
invMtrix = zeros(size(A,1), size(knowns,1));
j = 0;
for i=1:numNodes-1
    invMtrix(i,j+1:j+8) = [ 1 ,t(i),t(i)^2, t(i)^3, t(i)^4, t(i)^5, t(i)^6, t(i)^7];
    j = j+8;
end


j = 0;
for i=1:numNodes-1
    invMtrix(numNodes-1 + i, j+1:j+8) = [1 ,t(i+1),t(i+1)^2, t(i+1)^3,t(i+1)^4, t(i+1)^5 , t(i+1)^6, t(i+1)^7];
    j=j+8;
end

num = 2*numNodes -2;

% last two rows represent the velocities & accelerations  at the end and first point int the
% path, i.e. 0.

invMtrix(end,1:8 )=[0   ,  1   ,2*t(1)  , 3*t(1)^2 , 4*t(1)^3,  5*t(1)^4 ,  6*t(1)^5, 7*t(1)^6]; 
invMtrix(end-1,1:8 )=[ 0  ,   0   ,   2 ,    6*t(1) ,12*t(1)^2, 20*t(1)^3 , 5*6*t(1)^4, 6*7*t(1)^5]; 
invMtrix(end-2,1:8 )=[ 0  ,   0   ,   0,    6 , 2*12*t(1), 3*20*t(1)^2 , 4*5*6*t(1)^3, 5*6*7*t(1)^4]; 

invMtrix(end-3,end-7:end )=[ 0  ,   1 ,  2*t(end)  , 3*t(end)^2,  4*t(end)^3 , 5*t(end)^4, 6*t(end)^5, 7*t(end)^6];
invMtrix(end-4,end-7:end)=[ 0  ,   0 ,     2  ,6*t(end),12*t(end)^2 ,20*t(end)^3, 5*6*t(end)^4, 6*7*t(end)^5];
invMtrix(end-5,end-7:end)=[ 0  ,   0 ,     0  ,6 , 2*12*t(end) ,3*20*t(end)^2, 4*5*6*t(end)^3, 5*6*7*t(end)^4];

%% velocity at via points

% j = 0;
% k=2;
% for i=num +1:num + numNodes-2
%     
%     invMtrix(i,j+1:j+16) = [0   ,  1   ,2*t(k)  , 3*t(k)^2 , 4*t(k)^3,  5*t(k)^4 , 6*t(k)^5, 7*t(k)^6,...
%         0   ,  -1   ,-2*t(k)  , -3*t(k)^2, -4*t(k)^3,  -5*t(k)^4 , -6*t(k)^5, -7*t(k)^6];
%     k = k+1;
%     j = j+8;
% end

% %% accelerations at via points
% num2 = num + numNodes-2;
% j = 0;
% k = 2;
% for i=num2 +1:num2 + numNodes-2
%     
%     invMtrix(i,j+1:j+16) = [0   ,  0   ,2  , 6*t(k),12*t(k)^2 ,20*t(k)^3 ,5*6*t(k)^4, 6*7*t(k)^5, ...
%         0   ,  0   ,-2  , -6*t(k), -12*t(k)^2 ,-20*t(k)^3, -5*6*t(k)^4, -6*7*t(k)^5];
%     k = k+1;
%     j = j+8;
% end
% 
% %% jerks at via points
% num3 = num2 + numNodes-2;
% j = 0;
% k = 2;
% for i=num3 +1:num3 + numNodes-2
%     
%     invMtrix(i,j+1:j+16) = [0   ,  0   ,0  , 6, 2*12*t(k) ,3*20*t(k)^2, 4*5*6*t(k)^3, 5*6*7*t(k)^4,...
%         0   ,  0   ,0  , -6, -2*12*t(k) ,-3*20*t(k)^2,  -4*5*6*t(k)^3, -5*6*7*t(k)^4];
%     k = k+1;
%     j = j+8;
% end
% 
% %% snap at via points
% num4 = num3 + numNodes-2;
% j = 0;
% k = 2;
% for i=num4 +1:num4 + numNodes-2
%     
%     invMtrix(i,j+1:j+16) = [0   ,  0   ,0  , 0, 2*12 ,2*3*20*t(k) , 3*4*5*6*t(k)^2, 4*5*6*7*t(k)^3,...
%         0   ,  0   ,0  , 0, -2*12 ,-2*3*20*t(k) ,- 3*4*5*6*t(k)^2, -4*5*6*7*t(k)^3];
%     k = k+1;
%     j = j+8;
% end
% 
% %% crackle at via points
% num5 = num4 + numNodes-2;
% j = 0;
% k = 2;
% for i=num5 +1:num5 + numNodes-2
%     
%     invMtrix(i,j+1:j+16) = [0   ,  0   ,0  , 0,  0 ,2*3*20 , 2*3*4*5*6*t(k), 3*4*5*6*7*t(k)^2,...
%         0   ,  0   ,0  , 0, 0 ,-2*3*20 ,-2*3*4*5*6*t(k), -3*4*5*6*7*t(k)^2];
%     k = k+1;
%     j = j+8;
% end
% %% pop at via points
% num6 = num5 + numNodes-2;
% j = 0;
% k = 2;
% for i=num6 +1:num6 + numNodes-2
%     
%     invMtrix(i,j+1:j+16) = [0   ,  0   ,0  , 0,  0 , 0 , 2*3*4*5*6, 2*3*4*5*6*7*t(k),...
%         0   ,  0   ,0  , 0, 0 , 0 ,-2*3*4*5*6, -2*3*4*5*6*7*t(k)];
%     k = k+1;
%     j = j+8;
% end

coeffs = invMtrix\knowns;

a = coeffs;
end
