function writeFminconFiles(path,benchmark,Opts)
% WRITEFMINCONFILES - Generate files for optimal control using fmincon
%
% Syntax:
%       WRITEFMINCONFILES(path,benchmark,Opts)
%
% Description:
%       This function generates the cost function and the constraints for 
%       the optimal control problems as well as the corresponding 
%       derivatives using symbolic computing.
%
% Input Arguments:
%
%       -path:      path to the root directory of the control algorithm
%       -benchmark: name of the selected benchmark
%       -Opts:              a structure containing following options
%
%           -.nx:                   number of system states
%           -.nu:                   number of system inputs
%           -.extHorizon.active:    use extended optimization horizon for 
%                                   optimal control problems (optional)
%                                   [{false} / true]
%           -.extHorizon.horizon:   length of the extended optimization
%                                   horizon in center trajectory time steps
%                                   (optional) 
%                                   [{'all'} / positive integer]
%           -.extHorizon.decay:     decay function for the objective
%                                   function of the optimization problem
%                                   with extended optimization horizon
%                                   (optional)
%                                   [{'uniform'} / 'fall' / 'fall+end' / 
%                                    'fallLinear' / 'fallLinear+End' / 
%                                    'fallEqDiff' / 'FallEqDiff+End' / 
%                                    'rise' / 'quad' /  'riseLinear' /
%                                    'riseEqDiff' / 'end']
%
% See Also:
%       initOpts, writeAcadoFiles
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Synthesis developed at the Chair of Robotics, 
% Artificial Intelligence and Embedded Systems, 
% Technische Universitaet Muenchen. 
%
% For updates and further information please visit <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
%
% More Toolbox Info by searching <a href="matlab:docsearch aroc">AROC</a> in the Matlab Documentation
%
%------------------------------------------------------------------
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % parse options
    extHorizon.active = false;
    extHorizon.decay = 'uniform';
    extHorizon.horizon = 'all';
    
    if ~isfield(Opts,'extHorizon')
       Opts.extHorizon = extHorizon;
    else
        if ~isfield(Opts.extHorizon,'active')
           Opts.extHorizon.active = false; 
        end
        if ~isfield(Opts.extHorizon,'decay')
           Opts.extHorizon.decay = 'uniform'; 
        end
        if ~isfield(Opts.extHorizon,'horizon')
           Opts.extHorizon.horizon = 'all'; 
        end
    end
        
    % check if the system dynamics got changed
    generate = 0;
    filenameOld = [benchmark,'_fmincon_lastVersion.mat'];
    filePathLast = fullfile(path,'fmincon',benchmark,filenameOld);
    
    if ~exist(filePathLast,'file')
       generate = 1; 
    else
       load(filePathLast);
       equationNew = getSystemDynamics(benchmark,Opts);
       if equationOld ~= equationNew
           generate = 1;
       end
    end
    
    % delete all old files if the dynamics got changed
    if generate
       dirPath = fullfile(path,'fmincon',benchmark);
       if isdir(dirPath)
            rmdir(dirPath,'s');
       end
    end   

    % generate path for files
    pathNew = fullfile(path,'fmincon',benchmark,Opts.extHorizon.decay);
    
    % check if the folder exists and create it if not
    if ~exist(pathNew,'dir')
       mkdir(pathNew);
    end
    
    % calculate the necessary number of files 
    if Opts.extHorizon.active
       if ischar(Opts.extHorizon.horizon) && ...
                                    strcmp(Opts.extHorizon.horizon,'all')
          horizon = Opts.Nc; 
       else
          horizon = Opts.extHorizon.horizon;
       end
    else
       horizon = 1; 
    end
    
    steps = Opts.Ninter;
    
    % generate all files that do not exist yet
    for i = 1:horizon
        
       file1 = fullfile(pathNew,sprintf('fmincon_%i_%i_cost.m',steps,i));
       file2 = fullfile(pathNew,sprintf('fmincon_%i_%i_con.m',steps,i));
                                    
       if ~exist(file1,'file') || ~exist(file2,'file') 
          pathFile = fullfile(pathNew,sprintf('fmincon_%i_%i',steps,i));
          generateSymbolicFunctions(pathFile,benchmark,Opts,steps,i);
       end
    end
    
    % save dynamics from the currently generated files
    if generate
        equationOld = getSystemDynamics(benchmark,Opts);
        save(fullfile(path,'fmincon',benchmark,filenameOld),'equationOld');
    end
    
    % add the path with the files to the Matlab-path
    addpath(pathNew);
end


% Auxiliary Functions -----------------------------------------------------

function generateSymbolicFunctions(path,benchmark,Opts,steps,horizon)
% generate symbolic expressions for costs and constraints

    inter = 5;
    N = steps*horizon;

    % initialize symbolic variables
    x = sym('x',[Opts.nx,N*inter+1]);
    u = sym('u',[Opts.nu,N]);
    
    x0 = sym('x0',[Opts.nx,1]);
    xf = sym('xf',[Opts.nx,horizon]);
    Q = sym('Q',[Opts.nx,Opts.nx]);
    R = sym('R',[Opts.nu,Opts.nu]);
    dt = sym('dt');

    % constraints due to the system dynamics
    I = getIntegrator(benchmark,Opts.nw);

    ceq = [];
    cnt = 1;
    
    for i = 1:N
        for j = 1:inter
            ceq = [ceq; x(:,cnt+1) - I(x(:,cnt),u(:,i),dt/inter)];
            cnt = cnt + 1;
        end
    end
    
    % boundary conditions
    ceq = [ceq; x(:,1) - x0];
    
    % cost function
    decay = decayFunctions(horizon,Opts);
    cost = 0;
    
    for i = 1:horizon
        x_ = x(:,i*steps*inter) - xf(:,i);
        cost = cost + decay(i) * transpose(x_) * Q * x_; 
    end
    
    for i = 1:N
        cost = cost + dt * transpose(u(:,i)) * R * u(:,i);
    end
    
    % define optimization vector
    xOpt = [reshape(x,[numel(x),1]); reshape(u,[numel(u),1])];
    
    % compute derivatives
    costJac = jacobian(cost,xOpt);
    ceqJac = jacobian(ceq,xOpt);
    
    % write symbolic functions to files
    vars = {xOpt,x0,xf,Q,R,dt};

    matlabFunction(cost,costJac,'File',[path,'_cost.m'],'Vars',vars, ...
                                                         'Optimize',false);
    matlabFunction([],ceq,[],ceqJac,'File',[path,'_con.m'],'Vars', ...
                                                    vars,'Optimize',false);
end

function I = getIntegrator(benchmark,nw)
% define a symbolic function that returns the state after time dt

    % function handle to dynamic file
    str = ['funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);

    % integrator function
    I = @(x,u,dt) rungeKutta(funHandle,x,u,zeros(nw,1),dt);
end

function y = rungeKutta(f,x,u,w,dt)
% integration using Runge-Kutta-4

    k1 = f(x, u, w);
    k2 = f(x + dt*k1/2, u, w);
    k3 = f(x + dt*k2/2, u, w);
    k4 = f(x + dt*k3, u, w);
    
    y = x + 1/6 * (k1 + 2*k2 + 2*k3 + k4)*dt;
end

function f = getSystemDynamics(benchmark,Opts)
% get the system dynamics as a symbolic equation

    % function handle to dynamic file
    str = ['funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % symbolic expression
    x = sym('x',[Opts.nx,1]);
    u = sym('u',[Opts.nu,1]);
    w = sym('w',[Opts.nw,1]);

    f = funHandle(x,u,w);
end