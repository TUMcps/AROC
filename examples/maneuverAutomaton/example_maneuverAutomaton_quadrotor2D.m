% This example demonstrate how a maneuver automaton can be constructed and
% applied to solve an online control task for a 2D quadrotor.


% Generate Motion Primitives ----------------------------------------------

% load postprocessing function 
Post = @postprocessing_quadrotor2D;

% load system parameter
Params = param_quadrotor2D();

% define algorithm options
Opts = [];

Opts.N = 10;                        % number of time steps
Opts.Ninter = 2;                    % number of intermediate steps
Opts.reachSteps = 20;               % number of reachability steps
Opts.R = 1e-7*eye(2);               % input weighting matrix
Opts.extHorizon.active = true;      % use extended optimization horizon
Opts.extHorizon.decay = 'fall';     % time steps for ext. horizon
Opts.extHorizon.horizon = 5;        % weight function for extended horizon  

% define control inputs and initial states for motion primitives
list_xf = {[1.6;0;0;0;0;0],[-1.6;0;0;0;0;0],[1;1;0;0;0;0],[-1;1;0;0;0;0]};
list_Q = {[30,20,12,4,2,1],[30,20,12,4,2,1], ...
                                [82,12.8,10,11,2,1],[82,12.8,10,11,2,1]};

% loop over all motion primitives
primitives = {};
counter = 1;

for i = 1:length(list_xf)

    disp([newline,'Motion primitive ',num2str(counter), ...
         ' --------------',newline]);

    % update parameter and settings
    Params.xf = list_xf{i};
    Opts.Q = diag(list_Q{i});

    % compute controller for the current motion primitive
    [obj,res] = generatorSpaceControl('quadrotor2D',Params,Opts,Post);

    primitives{counter} = obj;
    counter = counter + 1;
end


% Construct Maneuver Automaton --------------------------------------------

% assemble input arguments
shiftFun = @shiftInitSet_quadrotor2D;
shiftOccFun = @shiftOccupancySet_quadrotor2D;

% construct maneuver automaton
MA = maneuverAutomaton(primitives,shiftFun,shiftOccFun);


% Planning Problem --------------------------------------------------------

% static obstacles
statObs{1} = polygon([0 0 3 3],[2.6 2.4 2.4 2.6]);
statObs{2} = polygon([-0.5 -0.5 1.5 1.5],[1.6 1.4 1.4 1.6]);
statObs{3} = polygon([1.5 1.5 2.5 2.5],[0.6 0.4 0.4 0.6]);
statObs{4} = polygon([2.6 2.6 3.7 3.7],[1.6 1.4 1.4 1.6]);

% goal set
goalSet = [];
goalSet.time = interval(0,4);
goalSet.set = polygon([2.5 2.5 3.5 3.5],[2.3 1.7 1.7 2.3]);

% initial set
x0 = [0;0;0;0;0;0];


% Online Control ----------------------------------------------------------

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet,statObs,[],'Astar');
tComp = toc(clock);

disp([newline,'Computation time (motion planning): ',num2str(tComp),'s']);


% Visualization -----------------------------------------------------------

% show the planned trajectory with an animation
speedUp = 0.5;

resSim = simulateRandom(MA,ind,x0,1);
animate(resSim,'quadrotor2D',statObs,[],goalSet.set,speedUp);

% plot planned trajectory
figure; hold on; box on;
plot(goalSet.set,[1,2],'FaceColor',[134 227 134]/255);
plotPlannedTrajectory(MA,ind,x0,[],[],'EdgeColor','k');
plotObstacles(statObs);