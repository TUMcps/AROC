% This example demonstrate how a Maneuver Automaton can be constructed and
% applied to solve an online control task for an autonomous ship.


% Generate Motion Primitives ----------------------------------------------

% load postprocessing function 
Post = @postprocessing_ship;

% load system parameter
Params = param_ship();

% define algorithm options
Opts = [];

Opts.N = 1;                              % number of time steps
Opts.Ninter = 35;                        % number of intermediate steps
Opts.reachSteps = 30;                    % number of reachability steps
Opts.Q = diag([1,1,110,10,10,20]);       % state weighting matrix
Opts.R = 0*eye(3);                       % input weighting matrix
Opts.refTraj.Q = diag([1 1 100 50 1 1]); % state weighting mat. (ref. traj)
Opts.cora.alg = 'poly';                  % reachability algorithm
Opts.cora.tensorOrder = 3;               % tensor order for reachability

% define final states for motion primitives
list_xf = {[150;-0.0203;-0.0005;5;0;0], ...
                [150;0.0203;0.0005;5;0;0],[150;0;0;5;0;0]};

% loop over all motion primitives
primitives = cell(length(list_xf),1);

for i = 1:length(list_xf)

    disp([newline,'Motion primitive ',num2str(i), ...
         ' --------------',newline]);

    % update parameter
    Params.xf = list_xf{i};
    
    % compute controller for the current motion primitive
    objContr = generatorSpaceControl('ship',Params,Opts,Post);

    primitives{i} = objContr;
end


% Construct Maneuver Automaton --------------------------------------------

% assemble input arguments
shiftFun = @shiftInitSet_ship;
shiftOccFun = @shiftOccupancySet_ship;

% construct maneuver automaton
MA = maneuverAutomaton(primitives,shiftFun,shiftOccFun);


% Online Control ----------------------------------------------------------

% load a CommonOcean scenario
scenario = 'USA_MEC-1_20190129_T-22';
[statObs,dynObs,x0,goalSet] = commonocean2cora(scenario);
x0 = [x0.x; x0.y; x0.orientation; x0.velocity; 0; 0];

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet{1},statObs,dynObs,'Astar',@costFun,@goalFun);
tComp = toc(clock);

disp([newline,'Computation time (motion planning): ',num2str(tComp),'s']);


% Visualization -----------------------------------------------------------

% show the planned trajectory with an animation
speedUp = 500;

resSim = simulateRandom(MA,ind,x0,1);
animate(resSim,'ship',[],dynObs,goalSet{1}.set,speedUp);

% visualize the planned trajectory for different times
figure; hold on; box on;
plot(goalSet{1}.set,[1,2],'FaceColor',[134 227 134]/255);
plotPlannedTrajectory(MA,ind,x0,interval(0,3000),'b');
plotObstacles([],dynObs,interval(0,3000));


% Auxiliary Functions -----------------------------------------------------

function res = goalFun(obj,goalSet,occSet,x,time)
% custom function to check if goal set is reached

    res = contains(goalSet.set,x(1:2)) & ...
                contains(goalSet.orientation,x(3));
end

function cost = costFun(obj,node,goalSet)
% compute the costs for A* star search for the current node

    % compute final state and final time at the end of the motion primitive
    index = node.ind(end);
    xCurr = obj.updateState(node.parent.xf,index); 
    time = node.parent.time + obj.primitives{index}.tFinal;
    
    % compute estimated remaining time for reaching the goal set
    c = center(goalSet.set);
    dist = sqrt(sum((c-xCurr(1:2)).^2));
    h = dist;
    g = time;
    
    % compute costs for the node
    cost = h + g;
end