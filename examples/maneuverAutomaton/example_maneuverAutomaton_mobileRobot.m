% This example demonstrate how a maneuver automaton can be constructed and
% applied to solve an online control task for a mobile robot.


% Generate Motion Primitives ----------------------------------------------

% load postprocessing function 
Post = @postprocessing_mobileRobot;

% load system parameter
Params = param_mobileRobot();

% define algorithm options
Opts = [];

Opts.N = 10;                          % number of time steps
Opts.Ninter = 2;                      % number of intermediate time steps
Opts.reachSteps = 5;                  % number of reachability steps
Opts.refTraj.Q = diag([1 1 1 5 5]);   % state weighting matrix (ref. traj.)
Opts.refTraj.R = diag([0.02 0.02]);   % input weighting matrix (ref. traj.)
Opts.parallel = 1;                    % use parallel computing
Opts.approx.method = 'optimized';     % method for approximating contr. law
Opts.extHorizon.active = true;        % use extended optimization horizon
Opts.extHorizon.horizon = 3;          % time steps for extended horizon
Opts.extHorizon.decay = 'fall+End';   % weight function for ext. horizon 

% define control inputs and initial states for motion primitives
list_x0 = {[0;0;0;1;1],[0;0;0;0;0],[0;0;0;1;1],[0;0;0;1;1]};
list_tFinal = {15,15,14,14};
list_xf = {[1;0;0;1;1],[1.5;0;0;1;1],[0.6;-0.6;-pi/2;1;1], ...
           [0.6;0.6;pi/2;1;1]};
list_Q = {[1 10 1 0.5 0.5],[1 1 1 1 1],[1 1 1 1 1],[1 1 1 1 1]};

% loop over all motion primitives
primitives = {};
counter = 1;

for i = 1:length(list_x0)

    disp([newline,'Motion primitive ',num2str(counter), ...
         ' --------------',newline]);

    % update parameter and settings
    Params.xf = list_xf{i};
    Params.R0 = Params.R0 + (-center(Params.R0)) + list_x0{i};
    Params.tFinal = list_tFinal{i};
    Opts.Q = diag(list_Q{i});

    % compute controller for the current motion primitive
    [obj,res] = convexInterpolationControl('mobileRobot',Params,Opts,Post);

    primitives{counter} = obj;
    counter = counter + 1;
end


% Construct Maneuver Automaton --------------------------------------------

% assemble input arguments
shiftFun = @shiftInitSet_mobileRobot;
shiftOccFun = @shiftOccupancySet_mobileRobot;

% construct maneuver automaton
MA = maneuverAutomaton(primitives,shiftFun,shiftOccFun);


% Planning Problem --------------------------------------------------------

% static obstacles
statObs{1} = interval([-0.3;-0.3],[-0.1;10.3]);
statObs{2} = interval([10.2;-0.3],[10.4;10.3]);
statObs{3} = interval([-0.3;-0.3],[10.4;-0.1]);
statObs{4} = interval([-0.3;10.1],[10.4;10.3]);
statObs{5} = interval([0.9;-0.1],[1.1;9]);
statObs{6} = interval([2.1;-0.1],[2.3;5]);
statObs{7} = interval([2.1;6],[2.3;10.1]);
statObs{8} = interval([3.3;-0.1],[3.5;9]);
statObs{9} = interval([4.5;1],[4.7;10.1]);
statObs{10} = interval([5.7;-0.1],[5.9;6]);
statObs{11} = interval([5.7;7],[5.9;10.1]);
statObs{12} = interval([6.9;1],[7.1;10.1]);
statObs{13} = interval([8;-0.1],[8.2;9.2]);
statObs{14} = interval([8.2;9],[9.3;9.2]);
statObs{15} = interval([8.2;6.5],[9.3;6.7]);
statObs{16} = interval([8.2;4],[9.3;4.2]);
statObs{17} = interval([9.3;7.7],[10.2;7.9]);
statObs{18} = interval([9.3;5.2],[10.2;5.4]);
statObs{19} = interval([9.3;2.7],[10.2;2.9]);

for i = 1:length(statObs)
   sup = supremum(statObs{i});
   infi = infimum(statObs{i});
   statObs{i} = polygon([infi(1) infi(1) sup(1) sup(1)], ...
                        [infi(2) sup(2) sup(2) infi(2)]);
end

% goal set
goalSet = [];
goalSet.time = interval(0,1000);
goalSet.set = polygon([8.7 8.7 9.7 9.7],[0 1 1 0]);

% initial set
x0 = [0.4;0.5;pi/2;0;0];


% Online Control ----------------------------------------------------------

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet,statObs,[],'depth-first');
tComp = toc(clock);

disp([newline,'Computation time (motion planning): ',num2str(tComp),'s']);


% Visualization -----------------------------------------------------------

% show the planned trajectory with an animation
speedUp = 10;

resSim = simulateRandom(MA,ind,x0,1);
animate(resSim,'mobileRobot',statObs,[],goalSet.set,speedUp);

% plot planned trajectory
figure; hold on; box on;
plot(goalSet.set,[1,2],'FaceColor',[134 227 134]/255);
plotPlannedTrajectory(MA,ind,x0,[],[],'EdgeColor','k');
plotObstacles(statObs);