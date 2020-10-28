%% Test Maneuver Automaton

% load postprocessing function 
Post = @postprocessing_car;

% load system parameter
Params = param_car();

% define algorithm options
Opts = settings_genSpaceContr_car();
Opts = rmfield(Opts,'refTraj');

% define control inputs and initial states for motion primitives
list_x0 = {[16.79;0;0;0]};
list_u1 = {0};
list_u2 = {-0.22:0.22:0.22};

% loop over all motion primitives
primitives = {};
counter = 1;

for i = 1:length(list_x0)

    % define ranges for inputs and get initial state
    [U1,U2] = meshgrid(list_u1{i},list_u2{i});

    %loop over the different control input combinations
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

% assemble input arguments
shiftFun = @shiftInitSet_car;
shiftOccFun = @shiftOccupancySet_car;

% construct maneuver automaton
MA = maneuverAutomaton(primitives,shiftFun,shiftOccFun);

% load a CommonRoad traffic scenario
scenario = 'USA_US101-6_2_T-1';
[statObs,dynObs,x0,goalSet,lanelets] = commonroad2cora(scenario);
x0 = [x0.velocity; x0.orientation; x0.x; x0.y];

% plan a verified trajectory with the maneuver automaton
clock = tic;
ind = motionPlanner(MA,x0,goalSet{1},statObs,dynObs,'depth-first');

% check if the end point of the simulation is inside the terminal region
ind_ = [3;1;2;2];
res = all(ind == ind_);
assert(res == 1);