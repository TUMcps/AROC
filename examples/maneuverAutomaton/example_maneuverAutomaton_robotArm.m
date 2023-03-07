% This example demonstrate how a maneuver automaton can be constructed and
% applied to solve an online control task for a mobile robot.


% Generate Motion Primitives ----------------------------------------------

% load postprocessing function 
Post = @postprocessing_robotArm;

% load system parameter
Params = param_robotArm();
Params.tFinal = 3;

% define algorithm options
Opts = [];

Opts.N = 10;                                % number of time steps
Opts.Ninter = 4;                            % number of intermediate steps
Opts.controller = 'linear';                 % controller
Opts.approx.method = 'scaled';              % method for approx. contr. law
Opts.Q = diag([10,10,1,1]);                 % state weighting matrix
Opts.R = diag([0;0]);                       % input weighting matrix
Opts.refTraj.Q = eye(4)./(0.05^2);          % state weighting (ref. traj.)
Opts.refTraj.R = diag([1;1]./([3;1].^2));   % input weighting (ref. traj.)


% define control inputs and initial states for motion primitives
list_x0 = {[0.1402;0.8957;0;0],[0.1402;0.8957;0;0],[0.1396;1.9552;0;0]};
list_xf = {[0.1396;1.9552;0;0],[0.865;0.855;0;0],[0.865;0.855;0;0]};

% loop over all motion primitives
primitives = {};
counter = 1;

for i = 1:length(list_x0)

    disp([newline,'Motion primitive ',num2str(counter), ...
         ' --------------',newline]);

    % update parameter and settings
    Params.xf = list_xf{i};
    Params.R0 = Params.R0 + (-center(Params.R0)) + list_x0{i};

    % compute controller for the current motion primitive
    [obj,res] = convexInterpolationControl('robotArm',Params,Opts,Post);

    primitives{counter} = obj;
    counter = counter + 1;
end


% Construct Maneuver Automaton --------------------------------------------

% assemble input arguments
shiftFun = @shiftInitSet_robotArm;
shiftOccFun = @shiftOccupancySet_robotArm;

% construct maneuver automaton
MA = maneuverAutomaton(primitives,shiftFun,shiftOccFun);


% Planning Problem --------------------------------------------------------

% static obstacles
statObs{1} = polygon([0.21 0.21 0.4 0.4],[0.3 0.5 0.5 0.3]);
statObs{2} = polygon([-0.3 -0.3 -0.05 -0.05],[0.15 0.4 0.4 0.15]);

% goal set
goalSet = [];
goalSet.time = interval(0,10);
goalSet.set = polygon([0.05 0.05 0.15 0.15],[0.3 0.4 0.4 0.3]);

% initial state
x0 = [0.1402;0.8957;0;0];


% Online Control ----------------------------------------------------------

% custom function to check if goal set is reached
goalFun = @(obj,goalSet,occSet,x,time) contains(goalSet.set, ...
                                   [0.2*cos(x(1)) + 0.2*cos(x(1)+x(2)); ...
                                    0.2*sin(x(1)) + 0.2*sin(x(1)+x(2))]);

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet,statObs,[],'depth-first',[],goalFun);
tComp = toc(clock);

disp([newline,'Computation time (motion planning): ',num2str(tComp),'s']);


% Visualization -----------------------------------------------------------

% show the planned trajectory with an animation
resSim = simulateRandom(MA,ind,x0,1);
animate(resSim,'robotArm',statObs,[],goalSet.set);

% plot planned trajectory
figure; hold on; box on;
plot(goalSet.set,[1,2],'FaceColor',[134 227 134]/255);
plotPlannedTrajectory(MA,ind,x0,[],[],'EdgeColor','k');
plotObstacles(statObs);