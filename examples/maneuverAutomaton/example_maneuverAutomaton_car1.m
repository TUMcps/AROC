% This example demonstrate how a Maneuver Automaton can be constructed and
% applied to solve an online control task for an autonomous car.


% Generate Motion Primitives ----------------------------------------------

% load postprocessing function 
Post = @postprocessing_car;

% load system parameter
Params = param_car();

% define algorithm options
Opts = [];

Opts.N = 5;                         % number of time steps
Opts.Ninter = 5;                    % number of intermediate time steps
Opts.extHorizon.active = 1;         % use extended optimization horizon
Opts.extHorizon.horizon = 5;        % time steps for ext. horizon
Opts.extHorizon.decay = 'fall';     % weight function for ext. horizon   

% define control inputs and initial states for motion primitives
list_x0 = {[23;0;0;0]; [16.93;0;0;0]; [12.33;0;0;0]};
list_u1 = {-6.07; -4.6; 0};
list_u2 = {-0.238; 0.246; -0.008:0.008:0};

% loop over all motion primitives
primitives = {};
counter = 1;

for i = 1:length(list_x0)

    % define ranges for inputs and get initial state
    [U1,U2] = meshgrid(list_u1{i},list_u2{i});

    % loop over the different control input combinations
    for j = 1:size(U1,1)
        for k = 1:size(U1,2)

            disp([newline,'Motion primitive ',num2str(counter), ...
                 ' --------------',newline]);
            
            % get reference trajectory by simulating the system
            x0 = list_x0{i};
            u = [U1(j,k); U2(j,k)];
            tspan = 0:Params.tFinal/(Opts.N*Opts.Ninter):Params.tFinal;
            fun = @(t,x) car(x,u,zeros(4,1));

            % get reference trajectory by simulating the system
            [t,x] = ode45(fun,tspan,x0);

            % provide reference trajectory as an additional input argument
            Opts.refTraj.x = x';
            Opts.refTraj.u = u*ones(1,size(x,1)-1);

            % update parameter
            Params.xf = x(end,:)';
            Params.R0 = Params.R0 + (-center(Params.R0)) + x0;
            
            % compute controller for the current motion primitive
            objContr = generatorSpaceControl('car',Params,Opts,Post);

            primitives{counter} = objContr;
            counter = counter + 1;
        end
    end
end


% Construct Maneuver Automaton --------------------------------------------

% assemble input arguments
shiftFun = @shiftInitSet_car;
shiftOccFun = @shiftOccupancySet_car;

% construct maneuver automaton
MA = maneuverAutomaton(primitives,shiftFun,shiftOccFun);


% Online Control ----------------------------------------------------------

% load a CommonRoad traffic scenario
scenario = 'ZAM_HW-1_1_S-1';
[statObs,dynObs,x0,goalSet,lanelets] = commonroad2cora(scenario);
x0 = [x0.velocity; x0.orientation; x0.x; x0.y];

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet{1},statObs,dynObs,'Astar');
tComp = toc(clock);

disp([newline,'Computation time (motion planning): ',num2str(tComp),'s']);


% Visualization -----------------------------------------------------------

% show the planned trajectory with an animation
resSim = simulateRandom(MA,ind,x0,1);
animate(resSim,'car',[],dynObs,goalSet{1}.set,[],lanelets);

% visualize the planned trajectory for different times
figure
times = {0,1,2,3};

for i = 1:length(times)
   subplot(length(times),1,i); hold on; box on;
   for j = 1:length(lanelets)
        plot(lanelets{j},[1,2],'FaceColor',[.6 .6 .6],'EdgeColor','k');
   end
   plotPlannedTrajectory(MA,ind,x0,[],[0 0.7 0],'EdgeColor','k');
   plotPlannedTrajectory(MA,ind,x0,interval(times{i}),'b');
   plotObstacles([],dynObs,interval(times{i}));
   xlim([0,70]);
end