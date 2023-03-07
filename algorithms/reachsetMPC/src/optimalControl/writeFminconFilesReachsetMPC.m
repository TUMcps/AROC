function writeFminconFilesReachsetMPC(path,benchmark,Opts)
% WRITEFMINCONFILESREACHSETMPC - Generate files for MPC optimal control 
%                                using fmincon
%
% Syntax:
%       WRITEFMINCONFILESREACHSETMPC(path,benchmark,Opts)
%
% Description:
%       This function generates the cost function and the constraints for 
%       the optimal control problems for Model Predictive Control as well 
%       as the corresponding derivatives using symbolic computing.
%
% Input Arguments:
%
%       -path:      path to the root directory of the control algorithm
%       -benchmark: name of the selected benchmark
%       -Opts:              a structure containing following options
%
%           -.nx:                   number of system states
%           -.nu:                   number of system inputs
%           -.nw:                   number of disturbances
%           -.N:                    number of time steps
%           -.Ninter:               number of intermediate time steps
%           -.termReg:              struct storing the properties of the
%                                   terminal region polytope
%
% See Also:
%       reachsetMPC, writeAcadoFilesMPC                                                                                             
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

    % number of inequality constraints for terminal region and state cons.
    M = size(Opts.termReg.A,1);

    if ~isempty(Opts.X)
        L = size(Opts.X.P.A,1);
    else
        L = 0;
    end
        
    % check if the system dynamics got changed
    generate = 0;
    filenameOld = [benchmark,'_fmincon_lastVersion.mat'];
    datanameOld = [benchmark,'_data_lastVersion.mat'];
    filePathLast = fullfile(path,'fmincon',benchmark,filenameOld);
    dataPathLast = fullfile(path,'fmincon',benchmark,datanameOld);
    
    if ~exist(filePathLast,'file') || ~exist(dataPathLast,'file')
       generate = 1; 
    else
       load(filePathLast);
       equationNew = getSystemDynamics(benchmark,Opts);
       if equationOld ~= equationNew
           generate = 1;
       else
           % check if parameters are identical
           load(dataPathLast);
           if data.M ~= M || data.L ~= L || data.N ~= Opts.N || ...
                                               data.Ninter ~= Opts.Ninter
              generate = 1; 
           end
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
    pathNew = fullfile(path,'fmincon',benchmark);
    
    % check if the folder exists and create it if not
    if ~exist(pathNew,'dir')
       mkdir(pathNew);
    end
    
    % generate all files that do not exist yet
    pathFile = strcat(pathNew,filesep,'fminconReachsetMPC');
    if ~exist([pathFile,'_cost'],'file') || ~exist([pathFile,'_con'],'file')
       generateSymbolicFunctions(pathFile,benchmark,Opts,Opts.N,Opts.Ninter,M,L);
    end
    
    % save dynamics and data from the currently generated files
    if generate
        
        % save system dynamics
        equationOld = getSystemDynamics(benchmark,Opts);
        save(filePathLast,'equationOld');
        
        % save data
        data.L = L;
        data.M = M;
        data.N = Opts.N;
        data.Ninter = Opts.Ninter;
        
        save(dataPathLast,'data');
    end
    
    % add the path with the files to the Matlab-path
    addpath(pathNew);
end


% Auxiliary Functions -----------------------------------------------------

function generateSymbolicFunctions(path,benchmark,Opts,N,Ninter,M,L)
% generate symbolic expressions for costs and constraints

    inter = 4;
    Nc = N*Ninter;

    % initialize symbolic variables
    x = sym('x',[Opts.nx,Nc*inter+1]);
    u = sym('u',[Opts.nu,Nc]);
    p = sym('p',[N,1]);
    
    x0 = sym('x0',[Opts.nx,1]);
    xf = sym('xf',[Opts.nx,1]);
    Q = sym('Q',[Opts.nx,Opts.nx]);
    R = sym('R',[Opts.nu,Opts.nu]);
    dt = sym('dt');
    D = sym('D',[M,Opts.nx]);
    e = sym('e',[M,1]);
    alpha = sym('a');
    alphaPoly = sym('ap');
    J = sym('J');
    u_min = sym('u_min',[Opts.nu,1]);
    u_max = sym('u_max',[Opts.nu,1]);

    if L > 0
        C = sym('C',[L,Opts.nx]);
        d = sym('d',[L,1]);
    end
    
    % constraints due to the system dynamics
    I = getIntegrator(benchmark,Opts.nw);

    ceq = [];
    cnt = 1;
    
    for i = 1:Nc
        for j = 1:inter
            ceq = [ceq; x(:,cnt+1) - I(x(:,cnt),u(:,i),dt/inter)];
            cnt = cnt + 1;
        end
    end
    
    % boundary conditions
    ceq = [ceq; x(:,1) - x0];
    
    % cost function
    cost = transpose(x(:,Nc*inter) - xf) * Q * (x(:,Nc*inter) - xf); 
    
    for i = 1:Nc
        cost = cost + dt * transpose(u(:,i)) * R * u(:,i);
    end

    cost = cost + 1e-6 * sum(p);
    
    % distance from polytope halfspaces
    e_ = alphaPoly * e;
    
    temp = cell(N,1);
    
    for i = 1:N
        temp{i} = (D*(x(:,i*Ninter*inter)-xf) - e_)./e_;
    end
    
    % contraction constraints
    c = [];
    
    for i = 1:N
       for j = 1:M
           c = [c; temp{i}(j) - p(i)];
       end
    end

    for j = 1:M
        c = [c; temp{N}(j)];
    end
    
    c = [c; sum(p) - J + alpha];

    % input constraints
    for i = 1:Nc
        c = [c; u_min - u(:,i)];
        c = [c; u(:,i) - u_max];
    end

    % state constraints
    if L > 0
        for i = 1:Nc*inter
            c = [c; C*x(:,i) - d];
        end
    end
    
    % define optimization vector
    xOpt = [reshape(x,[numel(x),1]); ...
            reshape(u,[numel(u),1]); ...
            reshape(p,[numel(p),1])];
    
    % compute derivatives
    costJac = jacobian(cost,xOpt);
    ceqJac = jacobian(ceq,xOpt);
    cJac = jacobian(c,xOpt);
    
    % write symbolic functions to files
    if L > 0
        vars = {xOpt,x0,xf,Q,R,dt,D,e,alpha,alphaPoly,J,u_min,u_max,C,d};
    else
        vars = {xOpt,x0,xf,Q,R,dt,D,e,alpha,alphaPoly,J,u_min,u_max};
    end

    matlabFunction(cost,costJac,'File',[path,'_cost.m'],'Vars',vars, ...
                                                         'Optimize',false);
    matlabFunction(c,ceq,cJac,ceqJac,'File',[path,'_con.m'],'Vars', ...
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