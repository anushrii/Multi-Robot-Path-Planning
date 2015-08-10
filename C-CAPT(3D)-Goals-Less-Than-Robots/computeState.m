function [pos,vel,acc] = computeState(currentcoeff,t)

ax = currentcoeff(:,1);
ay = currentcoeff(:,2);
az = currentcoeff(:,3);

x = ax(1) + ax(2)*t + ax(3)*t^2 + ax(4)*t^3 + ax(5)*t^4 + ax(6)*t^5 + ax(7)*t^6 + ax(8)*t^7;
y = ay(1) + ay(2)*t + ay(3)*t^2 + ay(4)*t^3 + ay(5)*t^4 + ay(6)*t^5 + ay(7)*t^6 + ay(8)*t^7;
z = az(1) + az(2)*t + az(3)*t^2 + az(4)*t^3 + az(5)*t^4 + az(6)*t^5 + az(7)*t^6 + az(8)*t^7;

dx = ax(2) + 2*ax(3)*t + 3*ax(4)*t^2 + 4*ax(5)*t^3 + 5*ax(6)*t^4 + 6*ax(7)*t^5 + 7*ax(8)*t^6;
dy = ay(2) + 2*ay(3)*t + 3*ay(4)*t^2 + 4*ay(5)*t^3 + 5*ay(6)*t^4 + 6*ay(7)*t^5 + 7*ay(8)*t^6;
dz = az(2) + 2*az(3)*t + 3*az(4)*t^2 + 4*az(5)*t^3 + 5*az(6)*t^4 + 6*az(7)*t^5 + 7*az(8)*t^6;

d2x = 2*ax(3) + 6*ax(4)*t + 4*3*ax(5)*t^2 + 5*4*ax(6)*t^3 + 5*6*ax(7)*t^4 + 6*7*ax(8)*t^5;
d2y = 2*ay(3) + 6*ay(4)*t + 4*3*ay(5)*t^2 + 5*4*ay(6)*t^3 + 5*6*ay(7)*t^4 + 6*7*ay(8)*t^5;
d2z = 2*az(3) + 6*az(4)*t + 4*3*az(5)*t^2 + 5*4*az(6)*t^3 + 5*6*az(7)*t^4 + 6*7*az(8)*t^5;

pos = [x; y;z];
vel = [dx; dy; dz];
acc = [d2x; d2y; d2z];

end