function writeFminconFilesConfChecking(benchmark,Opts)
% WRITEFMINCONFILESCONFCHECKING - Generate files required for conformant 
%                                 synthesis using fmincon
%
% Syntax:
%       WRITEFMINCONFILESCONFCHECKING(benchmark,Opts)
%
% Description:
%       This function generates the cost function and the constraints for 
%       conformant synthesis as well as the corresponding derivatives using 
%       symbolic computing
%
% Input Arguments:
%
%       -benchmark: name of the selected benchmark
%       -Opts:      a structure containing following options
%
%           -.group:   number of measurements grouped together
%           -.measErr: use measurement error to capture uncertainty
%           -.nx:      number of system states
%           -.nu:      number of system inputs
%
% See Also:
%       conformantSynthesis, conformantSynthesisNonlinear
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

    w = warning();
    warning('off','all');

    % file path where the files are generated
    [path,~,~] = fileparts(which('conformantSynthesis'));
    rmpath(genpath(fullfile(path,'fmincon')));
        
    % check if the system dynamics got changed
    generate = 0;
    
    if Opts.measErr
        id = ['Meas_',num2str(Opts.group)];  
    else
        id = ['No_',num2str(Opts.group)];
    end
    
    filenameOld = ['fmincon_lastVersion_',id,'.mat'];
    filePathLast = fullfile(path,'fmincon',benchmark,filenameOld);
    
    if ~exist(filePathLast,'file')
       generate = 1; 
    else
       load(filePathLast);
       equationNew = getSystemDynamics(benchmark,Opts);
       if equationOld ~= equationNew
           generate = 1;
       else
           acFileName1 = fullfile(path,'fmincon',benchmark, ...
                                    ['fminconConfChecking_',id,'_cost.m']);
           acFileName2 = fullfile(path,'fmincon',benchmark, ...
                                     ['fminconConfChecking_',id,'_con.m']);
           if ~exist(acFileName1,'file') || ~exist(acFileName2,'file')
              generate = 1; 
           end 
       end
    end

    % generate path for files
    pathNew = fullfile(path,'fmincon',benchmark);
    
    % check if the folder exists and create it if not
    if ~exist(pathNew,'dir')
       mkdir(pathNew);
    end
    
    % generate fmincon files
    if generate 
        pathFiles = fullfile(pathNew,['fminconConfChecking_',id]);
      	generateSymbolicFunctions(pathFiles,benchmark,Opts.nu,Opts.nx, ...
                                          Opts.nw,Opts.measErr,Opts.group);
    end
    
    % save dynamics from the currently generated files
    if generate
        equationOld = getSystemDynamics(benchmark,Opts);
        save(fullfile(path,'fmincon',benchmark,filenameOld),'equationOld');
    end
    
    % add the path with the files to the Matlab-path
    addpath(pathNew);
    
    warning(w);
end

function generateSymbolicFunctions(path,benchmark,nu,nx,nw,measErr,index)
% generate symbolic expressions for costs and constraints

    % initialize symbolic variables
    x = sym('x',[nx,index+1]);
    u = sym('u',[nu,index]);
    w = sym('w',[nw,index]);
    
    if measErr
       v = sym('v',[nx,index]); 
    end
    
    mu = sym('mu');
    dt = sym('dt',[index,1]);
    x0 = sym('x0',[nx,1]);
    xf = sym('xf',[nx,index]);

    % constraints due to the system dynamics
    I = getIntegrator(benchmark);

    ceq = [];
    
    for i = 1:index
       ceq = [ceq; x(:,i+1) - I(x(:,i),u(:,i),w(:,i),dt(i))]; 
    end
    
    % boundary conditions
    ceq = [ceq; x(:,1) - x0];
    
    for i = 1:index
        if measErr
            ceq = [ceq; x(:,i+1) + v(:,i) - xf(:,i)];
        else    
            ceq = [ceq; x(:,i+1) - xf(:,i)];
        end
    end
    
    % cost function
    cost = 0;
    
    for i = 1:index
       cost = cost + mu * transpose(w(:,i))*w(:,i); 
    end
    
    if measErr
        for i = 1:index
           cost = cost + (1-mu) * transpose(v(:,i))*v(:,i); 
        end
    end
    
    % define optimization vector
    xOpt = [reshape(x,[numel(x),1]); reshape(w,[numel(w),1])];
    
    if measErr
       xOpt = [xOpt; reshape(v,[numel(v),1])]; 
    end
    
    % compute derivatives
    costJac = jacobian(cost,xOpt);
    ceqJac = jacobian(ceq,xOpt);
    
    % write symbolic functions to files
    vars = {xOpt,x0,xf,u,dt,mu};

    matlabFunction(cost,costJac,'File',[path,'_cost.m'],'Vars',vars);
    matlabFunction([],ceq,[],ceqJac,'File',[path,'_con.m'],'Vars',vars);
end

function I = getIntegrator(benchmark)
% define a symbolic function that returns the state after time dt

    % function handle to dynamic file
    str = ['funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);

    % integrator function
    I = @(x,u,w,dt) rungeKutta(funHandle,x,u,w,dt);
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