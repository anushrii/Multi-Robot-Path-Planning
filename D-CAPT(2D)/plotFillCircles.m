function hs = plotFillCircles( x,y,r, col )
%PLOTFILLCIRCLES Summary of this function goes here
%   Detailed explanation goes here
n=length(x);
hs = cell(n,1);
for i = 1:n
    hs{i} = rectangle('Position',[x(i)-r(i),y(i)-r(i),2*r(i),2*r(i)],...
    'Curvature',[1,1],...
    'FaceColor',col);
end
axis equal
grid on
end

