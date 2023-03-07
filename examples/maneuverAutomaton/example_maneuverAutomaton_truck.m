% This example demonstrate how a Maneuver Automaton can be constructed and
% applied to solve an online control task for an autonomous truck.


% Generate Motion Primitives ----------------------------------------------

% load postprocessing function 
Post = @postprocessing_truck;

% load system parameter
Params = param_truck();
Params.R0 = Params.R0 + (-center(Params.R0)) + [0; 0; 0; 16.79; 0; 0];

% define algorithm options
Opts1 = [];

Opts1.N = 30;                           % number of time steps
Opts1.Ninter = 4;                       % number of intermediate steps
Opts1.reachSteps = 20;                  % number of reachability steps
Opts1.Q = diag([1,1,1,1,10,10]);        % state weighting matrix
Opts1.R = 1e-6*eye(2);                  % input weighting matrix

Opts2 = [];

Opts2.N = 10;                           % number of time steps
Opts2.Ninter = 5;                       % number of intermediate steps
Opts2.reachSteps = 20;                  % number of reachability steps
Opts2.Q = diag([2,14,12,1,10,40]);      % state weighting matrix
Opts2.R = 1e-6*eye(2);                  % input weighting matrix

% define control inputs and initial states for motion primitives
list_xf = {[0;0;0;16.79;50.37;-3.68], ...
                        [0;0;0;16.79;50.37;3.68],[0;0;0;16.79;16.79;0]};
list_tFinal = {3, 3, 1};
list_Opts = {Opts1, Opts1, Opts2};

% loop over all motion primitives
primitives = {};
counter = 1;

for i = 1:length(list_xf)

    disp([newline,'Motion primitive ',num2str(counter), ...
         ' --------------',newline]);

    % update parameter and settings
    Params.xf = list_xf{i};
    Params.tFinal = list_tFinal{i};
    Opts = list_Opts{i};

    % compute controller for the current motion primitive
    [obj,res] = generatorSpaceControl('truck',Params,Opts,Post);

    primitives{counter} = obj;
    counter = counter + 1;
end


% Construct Maneuver Automaton --------------------------------------------

% assemble input arguments
shiftFun = @shiftInitSet_truck;
shiftOccFun = @shiftOccupancySet_truck;

% construct maneuver automaton
MA = maneuverAutomaton(primitives,shiftFun,shiftOccFun);


% Online Control ----------------------------------------------------------

% load a CommonRoad traffic scenario
scenario = 'USA_US101-6_2_T-1';
[statObs,dynObs,x0,goalSet,lanelets] = commonroad2cora(scenario);
x0 = [0; x0.orientation; 0; x0.velocity; x0.x-0.1; x0.y];
goalSet{1}.time = interval(3,4);

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet{1},[],dynObs,'depth-first');
tComp = toc(clock);

disp([newline,'Computation time (motion planning): ',num2str(tComp),'s']);


% Visualization -----------------------------------------------------------

% show the planned trajectory with an animation
resSim = simulateRandom(MA,ind,x0,1);
animate(resSim,'truck',[],dynObs,goalSet{1}.set,[],lanelets);

% visualize the planned trajectory for different times
figure
times = {0,1.5,3};

for i = 1:length(times)
   subplot(1,length(times),i); hold on; box on;
   for j = 1:length(lanelets)
        plot(lanelets{j},[1,2],'FaceColor',[.6 .6 .6],'EdgeColor','k');
   end
   plotPlannedTrajectory(MA,ind,x0,[],[0 0.7 0],'EdgeColor','k');
   plotPlannedTrajectory(MA,ind,x0,interval(times{i}),'b');
   plotObstacles([],dynObs,interval(times{i}-0.05,times{i}+0.05));
   xlim([-10,60]); ylim([-100,40]);
end