%% Clear the previous
clear all
close all

%% Parameters
timeTaken = zeros(1,10);
avg_time =zeros(1,10);
std_time =zeros(1,10);
figure,

xlabel('Number of Robots')
ylabel('Time to Compute(s)')
for j =1:10
    
    var.nbots = 100*j;
    var.ngoals = 100*j;
    
    for i = 1: 10
        display(j)
        display(i)
        
        tic
        % num robots and goals
        
        
        % num dimensions
        var.n = 2;
        
        % radius
        var.R = 0.3;
        
        % boundary x axis and y axis
        var.bound = 10000;
        
        % max vel
        var.vmax = 30;
        
        % st time
        var.t0 = 0;
        
        %% Assign the goals
        
        [start,goal] = generateStart_Goal(var);
        
        % compute the assignment, the time parameterization and
        % the coffiecients to compute the state at every instant
        % first time to get tf or in case to get init N>=M
        var = init(start,goal,var);
        
        t_step = 0.1;
        
        % the radius of the robot
        currentgoals = goal;
        currentstate = start;
        
        
        
        
        %% If there are more goals than goals, run the algorithm iteratively
        % first assigning all the robots to some goals. Remove those goals and
        % run again.
        while size(currentgoals,1)>var.nbots
            % run simulation
            t_disc = [var.t0:t_step:var.t0+var.tf,var.tf];
            for it = 1:numel(t_disc)
                
                t = t_disc(it);
                state = computeCAPT(var,t);
                
                
            end
            
            % remove the visited goals
            visitedgoals = sum(var.phi,1);
            currentgoals = currentgoals((~visitedgoals),:);
            var.t0 = 0;
            currentstate = state;
            var = init(currentstate,currentgoals,var);
        end
        
        %% Run the last time (if M>N) or for the first time
        
        for t=0:t_step:var.tf
            state = computeCAPT(var,t);
        end
        
        timeTaken(i) = toc;
        
        
        
    end
    
    avg_time(j)  = mean(timeTaken);
    
    loglog(var.nbots,avg_time(j),'.-')
    hold on
    drawnow
    std_time(j)  = std(timeTaken);
end

%%