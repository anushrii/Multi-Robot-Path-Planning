function plotCircles(x,y,r)
% plotting any number of circles
% x y r N by 1
theta=[0:0.01:2*pi]; 
xp=r*cos(theta);
yp=r*sin(theta);
xf = bsxfun(@plus, xp, x);
yf = bsxfun(@plus, yp, y);
plot(xf',yf','m');
axis equal
end