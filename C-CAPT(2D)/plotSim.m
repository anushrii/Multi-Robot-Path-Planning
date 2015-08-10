function plotSim(state, var, start, goal, scale)
%% Function to plot the simulation

% draw robot
h = plot(state(:,1),state(:,2),'bo',...
    state(:,1),state(:,2),'ro');
% plot the radius
set(h,{'markers'},{20;2*1.414*var.R*scale/2});
set(h,{'markerfacecolor'},{'y';'r'});
daspect([1 1 1]);
set(gcf,'units','inches')
hold on

% set the boundary
axis([0 var.bound 0 var.bound])
grid on
plot(start(:,1),start(:,2),'go','MarkerSize',20)
plot(goal(:,1),goal(:,2),'ko','MarkerSize',20)
hold off
drawnow;
end