% This example demonstrate how a Maneuver Automaton can be constructed and
% applied to solve an online control task.


% Generate Motion Primitives ----------------------------------------------

% load postprocessing function 
Post = @postprocessing_car;

% load system parameter
Params = param_car();

% define algorithm options
Opts = settings_genSpaceContr_car();
Opts = rmfield(Opts,'refTraj');

% define control inputs and initial states for motion primitives
list_x0 = {[15.8773;0;0;0];[14.8773;0;0;0];[14.8773;0;0;0]};
list_u1 = {-1;0;0};
list_u2 = {0;-0.15:0.15:0;0.18};

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
scenario = 'ZAM_Zip-1_19_T-1';
[statObs,dynObs,x0,goalSet,lanelets] = commonroad2cora(scenario);
x0 = [x0.velocity; x0.orientation; x0.x; x0.y];

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet{1},statObs,dynObs,'Astar');
tComp = toc(clock);

disp([newline,'Computation time (motion planning): ',num2str(tComp),'s']);


% Visualization -----------------------------------------------------------

% show the planned trajectory with an animation
animateCommonRoad(MA,ind,dynObs,x0,goalSet{1},lanelets);

% visualize the planned trajectory for different time intervals
figure
timeInt = {[0,1],[2,3],[4,5],[6,7]};

for i = 1:length(timeInt)
   subplot(length(timeInt),1,i);
   time = timeInt{i};
   visualizeCommonRoad(MA,ind,dynObs,x0,lanelets,interval(time(1),time(2)));
   xlim([-150,50]);
   ylim([-10,20]);
   title(['$t \in [',num2str(time(1)),',',num2str(time(2)),']s$'], ...
         'interpreter','latex');
end