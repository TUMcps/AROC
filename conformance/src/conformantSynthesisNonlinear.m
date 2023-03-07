function [W,V] = conformantSynthesisNonlinear(benchmark,measurements,Opts)
% CONFORMANTSYNTHESISNONLINEAR - conformant synthesis for nonlinear models
%
% Syntax:
%       [W,V] = CONFORMANTSYNTHESISNONLINEAR(benchmark,measurements,Opts)
%
% Description:
%       This function computes the model uncertainty that is required such 
%       that the uncertain model encloses all measurements of the real 
%       system. The model uncertainty consists of a set of uncertain inputs
%       W and a set of measurement errors V.
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -measuremnts:  cell-array storing the measurements of the real
%                      system, where each cell is a struct with the 
%                      following fields: 
%
%           -.x:       matrix storing the measured trajectory of the real
%                      system (dimension: [n,N])
%           -.u:       matrix storing the input signal corresponding to the
%                      measured trajectory (dimension: [m,N-1])
%           -.t:       vector storing the time points for the measured
%                      trajectory (dimension: [1,N])
%
%       -Opts:         a structure containing the algorithm settings
%
%           -.set:     string specifying the type of set which is used to
%                      represent the uncertainty.
%                      [{'interval'} / 'zonotope']
%           -.group:   number of measurements grouped together
%                      [{1} / positive integer]
%           -.measErr: use measurement error to capture uncertainty
%                      [{false} / true]
%           -.mu       tradeoff between uncertain inputs and measurement
%                      errors
%                      [{0.5} / double between 0 and 1]
%
% Output Arguments:
%
%       -W:     resulting set of uncertain inputs
%       -V:     resulting set of measurement errors
%
% See Also:
%       conformantSynthesis, conformantSynthesisLinear
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

    % function handle to the dynamic file
    str = ['funHan = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);

    % check if ACADO is available or not
    useAcado = 1;
    if isempty(which('BEGIN_ACADO'))
        useAcado = 0;
    end

    % store data in options
    Opts.useAcado = useAcado;
    Opts.funHan = funHan;
    
    % geneate filed for ACADO toolbox
    if useAcado
        writeAcadoFilesConfChecking(benchmark,Opts);
    else
        writeFminconFilesConfChecking(benchmark,Opts);
    end
        
    % loop over all measurements and compute uncertainty
    W = zeros(Opts.nw,1); V = zeros(Opts.nx,1);
    
    for i = 1:length(measurements)

        M = measurements{i};
        traj = M.x(:,1);
        batch_start = 2:Opts.group:size(M.x,2);
        batch_end = batch_start + Opts.group-1;
        if batch_end(end) >= size(M.x,2)
           batch_start(end) = size(M.x,2)-Opts.group+1;
           batch_end(end) = size(M.x,2);
        end

        for j = 1:length(batch_start)

            % compute uncertain input for a single measurement
            x = M.x(:,batch_start(j):batch_end(j));
            u = M.u(:,batch_start(j)-1:batch_end(j)-1);
            x0 = traj(:,batch_start(j)-1);
            tmp = M.t(batch_start(j)-1:batch_end(j));
            dt = diff(tmp);
            
            if Opts.useAcado
                if Opts.measErr
                    [w,v,tmp] = compUncertaintyACADO(x0,x,u,dt,Opts);
                else
                    [w,tmp] = compUncertainInputACADO(x0,x,u,dt,Opts);                    
                end
            else
                if Opts.measErr
                    [w,v,tmp] = compUncertaintyFmincon(x0,x,u,dt,Opts);
                else
                    [w,tmp] = compUncertainInputFmincon(x0,x,u,dt,Opts);
                end
            end
            
            % update initial state
            traj = [traj, tmp];

            % update set of uncertain inputs
            if strcmp(Opts.set,'interval')
                W = W | interval(w);
                if Opts.measErr
                   V = V | interval(v); 
                end
            else
                W = [W,w]; 
                if Opts.measErr
                   V = [V,v]; 
                end
            end
        end                        
    end
    
    % enclose point cloud with a zonotope
    if ~strcmp(Opts.set,'interval')
       W = zonotope.enclosePoints(W,'maiga'); 
       if Opts.measErr
          V = zonotope.enclosePoints(V,'maiga'); 
       end
    end
end


% Auxiliary Functions -----------------------------------------------------

function [w,x_] = compUncertainInputACADO(x0,x,u,dt,Opts)
% compute the set of required uncertain inputs using ACADO

    % passing optimization variables to ACADO Toolbox, every scalar has to
    % be passed individually
    x0_ = num2cell(x0);
    x_ = num2cell(reshape(x,[numel(x),1]));
    u_ = num2cell(reshape(u,[numel(u),1]));
    dt_ = num2cell(max(dt)./dt);

    text = sprintf('out = acadoConfChecking_No_%i_RUN(x0_{:},x_{:},u_{:},dt_{:},max(dt),Opts.mu);',Opts.group);
    evalc(text);
    
    w = out.CONTROLS(2,2:end)';
    w = reshape(w,[numel(w)/Opts.group,Opts.group]);
    x_ = x0;
    
    for i = 1:size(w,2)
        [~,traj] = ode45(@(t,x_)Opts.funHan(x_,u(:,i),w(:,i)),[0 dt(i)],x_(:,end));
        x_ = [x_, traj(end,:)'];
    end
    x_ = x_(:,2:end);
    
    if max(max(abs(x_-x))) > 1e-3
        error('Conformance checking failed!');
    end
end
function [w,v,x_] = compUncertaintyACADO(x0,x,u,dt,Opts)
% compute the set of required uncertain inputs and measurement errors using 
% ACADO

    % passing optimization variables to ACADO Toolbox, every scalar has to
    % be passed individually
    x0_ = num2cell(x0);
    x_ = num2cell(reshape(x,[numel(x),1]));
    u_ = num2cell(reshape(u,[numel(u),1]));
    dt_ = num2cell(max(dt)./dt);

    text = sprintf('out = acadoConfChecking_Meas_%i_RUN(x0_{:},x_{:},u_{:},dt_{:},max(dt),Opts.mu);',Opts.group);
    evalc(text);
    
    tmp = out.CONTROLS(2,2:end)';
    v = tmp(end-Opts.group*length(x0)+1:end);
    w = tmp(1:end-Opts.group*length(x0));
    w = reshape(w,[numel(w)/Opts.group,Opts.group]);
    v = reshape(v,[numel(v)/Opts.group,Opts.group]);
    x_ = x0;
    
    for i = 1:size(w,2)
        [~,traj] = ode45(@(t,x_)Opts.funHan(x_,u(:,i),w(:,i)),[0 dt(i)],x_(:,end));
        x_ = [x_, traj(end,:)'];
    end
    x_ = x_(:,2:end);
    
    if max(max(abs(x_+v-x))) > 1e-3
        error('Conformance checking failed!');
    end
end

function [w,x_] = compUncertainInputFmincon(x0,xf,u,dt,Opts)
% compute the set of required uncertain inputs using fmincon
    
    % solve optimal control problem
    dt = dt';

    text = sprintf(['cost = @(x) fminconConfChecking_No_%i_cost', ...
                                    '(x,x0,xf,u,dt,Opts.mu);'],Opts.group);
    eval(text);
    
    text = sprintf(['con = @(x) fminconConfChecking_No_%i_con', ...
                                    '(x,x0,xf,u,dt,Opts.mu);'],Opts.group);
    eval(text);
    
    options = optimoptions('fmincon','Algorithm', ...
                            'interior-point','Display','off'); 
                        
    nOpt = Opts.nx*(Opts.group+1) + Opts.group*Opts.nw;
    xInit = zeros(nOpt,1);
    
    sol = fmincon(cost,xInit,[],[],[],[],[],[],con,options);

    % check the result
    x = sol(1:Opts.nx*(Opts.group+1));
    x = reshape(x,[Opts.nx,Opts.group+1]);
    w = sol(numel(x)+1:numel(x)+Opts.group*Opts.nw);
    w = reshape(w,[Opts.nw,Opts.group]);
    x_ = x0;
    
    for i = 1:size(w,2)
        [~,traj] = ode45(@(t,x_)Opts.funHan(x_,u(:,i),w(:,i)), ...
                                                    [0 dt(i)],x_(:,end));
        x_ = [x_, traj(end,:)'];
    end
    x_ = x_(:,2:end);
    
    if max(max(abs(x_-xf))) > 1e-3
        error('Conformance checking failed!');
    end
end

function [w,v,x_] = compUncertaintyFmincon(x0,xf,u,dt,Opts)
% compute the set of required uncertain inputs using fmincon
    
    % solve optimal control problem
    dt = dt';

    text = sprintf(['cost = @(x) fminconConfChecking_Meas_%i_cost', ...
                                    '(x,x0,xf,u,dt,Opts.mu);'],Opts.group);
    eval(text);
    
    text = sprintf(['con = @(x) fminconConfChecking_Meas_%i_con', ...
                                    '(x,x0,xf,u,dt,Opts.mu);'],Opts.group);
    eval(text);
    
    options = optimoptions('fmincon','Algorithm', ...
                            'interior-point','Display','off'); 
                        
    nOpt = Opts.nx*(Opts.group+1) + Opts.group*(Opts.nw + Opts.nx);
    xInit = zeros(nOpt,1);
    
    sol = fmincon(cost,xInit,[],[],[],[],[],[],con,options);

    % check the result
    x = sol(1:Opts.nx*(Opts.group+1));
    x = reshape(x,[Opts.nx,Opts.group+1]);
    w = sol(numel(x)+1:numel(x)+Opts.group*Opts.nw);
    w = reshape(w,[Opts.nw,Opts.group]);
    v = sol(numel(x)+numel(w)+1:end);
    v = reshape(v,[Opts.nx,Opts.group]);
    x_ = x0;
    
    for i = 1:size(w,2)
        [~,traj] = ode45(@(t,x_)Opts.funHan(x_,u(:,i),w(:,i)), ...
                                                    [0 dt(i)],x_(:,end));
        x_ = [x_, traj(end,:)'];
    end
    x_ = x_(:,2:end);
    
    if max(max(abs(x_+v-xf))) > 1e-3
        error('Conformance checking failed!');
    end
end