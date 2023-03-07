%% Test Nonlinear Conformance Checking

% define parameter
X0 = interval([1;0;0;0],[30;2*pi;100;100]);     % domain initial points
W = interval([-0.5;-0.02],[0.5;0.02]);          % set of disturbances
U = interval([-9.81;-0.4],[9.81;0.4]);          % sef of contol inputs
tFinal = 1;                                     % final time
N = 3;                                          % number of measurements
changes = 10;                                   % number of dist. changes
sampTime = 0.01;                                % sampling time

% generare measurements by simulation
M = cell(N,1);

for i = 1:N
    
    % select random initial point and control input
    x0 = randPoint(X0);
    u_ = randPoint(U);
    t0 = 0;
    
    x = x0; u = []; t = t0; dt = tFinal/changes;
    
    for j = 1:changes
        
        % select random disturbance value
        w = randPoint(W);
        
        % simulate the system
        [tTemp,xTemp] = ode45(@(t,x)car(x,u_,w),t0:sampTime:t0 + dt,x0);
        
        % store the results 
        x = [x, xTemp(2:end,:)'];
        t = [t, tTemp(2:end)'];
        u = [u, u_*ones(1,length(tTemp)-1)];
        
        % update time and initial state
        t0 = t(end); x0 = x(:,end);
    end
    
    % store results in measurements
    M{i}.x = x;
    M{i}.u = u;
    M{i}.t = t; 
end

% conformance checking
Opts = [];
Opts.group = 10;
Opts.measErr = true;

[W,V] = conformantSynthesis('car',M,Opts);

% check if the reachable set contains all measurements
for i = 1:length(M)
    R = reachSetConformance('car',M{i},W,V);
    for j = 1:length(R.timePoint.set)
        assert(containsFast(R.timePoint.set{j}, M{i}.x(:,j)))
    end
end


%% Test Linear Conformance Checking

% define parameter
x0 = [0;20;1;0;1;0;1;0];
width = [0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2];

X0 = interval(x0-width,x0+width);               % domain initial points
W = interval(-[1;1;1;1],[1;1;1;1]);             % set of disturbances
U = interval(-[10;10;10;10],[10;10;10;10]);     % sef of contol inputs
tFinal = 1;                                     % final time
N = 3;                                          % number of measurements
changes = 10;                                   % number of dist. changes
sampTime = 0.01;                                % sampling time

% generare measurements by simulation
M = cell(N,1);

for i = 1:N
    
    % select random initial point and control input
    x0 = randPoint(X0);
    u_ = randPoint(U);
    t0 = 0;
    
    x = x0; u = []; t = t0; dt = tFinal/changes;
    
    for j = 1:changes
        
        % select random disturbance value
        w = randPoint(W);
        
        % simulate the system
        [tTemp,xTemp] = ode45(@(t,x)platoon(x,u_,w),t0:sampTime:t0 + dt,x0);
        
        % store the results 
        x = [x, xTemp(2:end,:)'];
        t = [t, tTemp(2:end)'];
        u = [u, u_*ones(1,length(tTemp)-1)];
        
        % update time and initial state
        t0 = t(end); x0 = x(:,end);
    end
    
    % store results in measurements
    M{i}.x = x;
    M{i}.u = u;
    M{i}.t = t; 
end

% conformance checking
Opts.group = 10;
Opts.measErr = true;

[W,V] = conformantSynthesis('platoon',M,Opts);

% check if the reachable set contains all measurements
for i = 1:length(M)
    R = reachSetConformance('platoon',M{i},W,V);
    for j = 1:length(R.timePoint.set)
        assert(containsFast(R.timePoint.set{j}, M{i}.x(:,j)))
    end
end