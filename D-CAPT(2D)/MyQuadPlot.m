classdef MyQuadPlot
    %Simple 2D plot of quad rotors and their positionss
    
    properties
        quadInnerCircles
        quadHCircles
        goalsCircles
        % radius of robot
        r
        % field
        h
        % num robots
        N
        goal
        bound
    end
    
    methods
        function obj = MyQuadPlot(start, goal, r,  bound ,h)
            obj.r = r;
            if nargin==5
                obj.h = h;
            else
                obj.h = r;
            end
            obj.N = size(start,1);
            obj.goal = goal;
            obj.quadHCircles = plotFillCircles(start(:,1),start(:,2),ones(obj.N,1)*h,'y');
            obj.goalsCircles = plotFillCircles(goal(:,1),goal(:,2),ones(obj.N,1)*r,'g'); 
            obj.quadInnerCircles = plotFillCircles(start(:,1),start(:,2),ones(obj.N,1)*r,'r');
            obj.bound = bound;
            axis([0 obj.bound 0 obj.bound])
            drawnow;
        end
        
        function update(obj, X)
            % X new positions N by 2
            
            for i = 1:length(obj.quadHCircles)
                set(obj.quadHCircles{i}, 'Position', [X(i,1)-obj.h, X(i,2)-obj.h, 2*obj.h, 2*obj.h]);  
            end
            for i = 1:length(obj.quadHCircles)
                set(obj.quadInnerCircles{i}, 'Position', [X(i,1)-obj.r, X(i,2)-obj.r, 2*obj.r, 2*obj.r]);
            end
            goal = obj.goal;
% %             plotFillCircles(goal(:,1),goal(:,2),ones(obj.N,1)*obj.r,'g');
            
%             plotCircles(X(:,1),X(:,2),obj.r);
             hold on;
             plot(X(:,1),X(:,2),'g.');
       
             plotFillCircles(goal(:,1),goal(:,2),ones(obj.N,1)*obj.r*6,'g');
             axis([0 obj.bound 0 obj.bound  ])
             box on
            drawnow;
        end
    end
    
end

