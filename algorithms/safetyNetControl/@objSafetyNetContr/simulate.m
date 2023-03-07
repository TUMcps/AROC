function [res,t,x,u,flag] = simulate(obj,x0,w,varargin)
% SIMULATE - simulate a trajectory of a nonlinear system controlled with 
%            the Safety Net Controller
%
% Syntax:
%       [res,t,x,u,flag] = SIMULATE(obj,x0,w)
%       [res,t,x,u,flag] = SIMULATE(obj,x0,w,v)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with the Safety Net Controler for a given initial point and given 
%       specific disturbance values over time.
%
% Input Arguments:
%
%       -obj:   object of class objConvInterContr storing the control law
%               computed in the offline-phase
%       -x0:    initial point for the simulation
%       -w:     matrix storing the values for the disturbances over time
%               (dimension: [nw,multiple of N*Ninter])
%       -v:     matrix storing the values for the measurement errors over
%               time (dimension: [nx,multiple of N*Ninter]) 
%
% Output Arguments:
%
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the applied control input 
%               (dimension: [|t|,nu])
%       -flag:  vector storing a flag that specifies which controller was
%               active (1: comfort controller, 0: safety net controller)
%
% See Also:
%       safetyNetControl, simulateRandom
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
% Authors:      Moritz Klischat, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

    % check if the number of specified disturbance vectors is correct
    if size(w,2) > 1
        Nw = size(w,2)/(obj.N * obj.Ninter);
        if Nw < 1 || floor(Nw) ~= Nw
           error('Number of disturbance vectors has to be a multiple of ''N*Ninter''!'); 
        end
    end
    
    % check if the number of specified measurement errors is correct
   	if nargin > 3
       v = varargin{1};
       if ~all(size(v) == [obj.nx,size(w,2)])
           error('Wrong dimension for the measurement errors "v"!'); 
       end
    else
       v = zeros(obj.nx,size(w,2));
    end
    
    % initialize comfort controllers
    for i = 1:length(obj.comfContr)
       init(obj.comfContr{i}); 
    end
    initSafetyNet(obj);
    
    % initialization
    x_ = x0;
    Rpred = zonotope(x0);
    x = []; u = []; t = []; flag = [];
    counter = 1; tComp1 = 0;
    
    % loop over all time steps
    for i = 1:obj.N
        
        wTemp = w(:,counter:counter+obj.Ninter*Nw-1);
        vTemp = v(:,counter:counter+obj.Ninter*Nw-1);
        comfFeas = 0;
        
        % loop over all parallel comfort controller
        for j = 1:length(obj.comfContr)
        
            % compute the reachable set of the comfort controller
            clock = tic;
            [conSat,R,Param] = reachSet(obj.comfContr{j},Rpred,i);
            tComp2 = toc(clock);

            % check if the input constraints are satisfied and the reachable 
            % set of the comfort controller is located inside the reachable
            % set of the safety net controller       
            if conSat == 1 && contains(obj.reachSet{i+1},R) && ...
               (~obj.realTime || tComp1+tComp2 <= obj.tComp)
                % comfort controller feasible -> execute comfort controller
                [tTemp,xTemp,uTemp] = simulate(obj.comfContr{j},x_, ...
                                                      wTemp,vTemp,Param,i);
                fTemp = ones(size(tTemp));
                comfFeas = 1; indComf = j;
                break;
            end
        end
        
        % no comfort controller feasible -> execute safety net controller
        if ~comfFeas
            [tTemp,xTemp,uTemp,alpha] = simulateSafetyNet(obj,x_,wTemp, ...
                                                                  vTemp,i);
            fTemp = zeros(size(tTemp));
        end
        
        % predict states at the end of the allocated computation time 
        [~,ind] = min((tTemp-obj.tComp).^2);
        xMeas = xTemp(ind(1),:)'+ vTemp(:,obj.Npred*Nw);
        
        clock = tic;
        if comfFeas
            Rpred = reachSetPred(obj.comfContr{indComf},xMeas,i,Param);
        else
            Rpred = reachSetPredSafetyNet(obj,xMeas,alpha,i);
        end
        tComp1 = toc(clock);
        
        % store the simulation results
        x_ = xTemp(end,:)'; x = [x;xTemp]; u = [u;uTemp]; 
        flag = [flag;fTemp];
        if isempty(t)
            t = [t;tTemp];
        else
            t = [t;t(end)+tTemp];
        end
        counter = counter + obj.Ninter;
    end

    % store simulation in results object
    sim{1}.t = t;
    sim{1}.x = x;
    sim{1}.u = u;
    res = results([],[],[],sim);
end


% Auxiliary Functions -----------------------------------------------------

function [t,x,u,alpha] = simulateSafetyNet(obj,x0,w,v,iter)
% simulate the trajecoty for the safety net controller for one time step

    U = obj.Usub{iter};
    H = obj.H{iter};
    x_ = x0;
    Nw = size(w,2);
    nu = dim(U{1});
    timeStep = obj.tFinal/(obj.N*obj.Ninter*Nw);
    x = []; t = []; u = [];

    % solve linear program to obtain the value for the factors alpha
    alpha = getFactorValues(obj,x0+v(:,1),iter);

    % loop over all intermediate time steps
    for i = 1:obj.Ninter
       
        % compute control input
        cu = center(U{i});
        Gu = generators(U{i});
        
        ind = (obj.Ninter-i)*nu;
        H_ = H(ind+1:ind+nu,:);
        
        u_ = cu + Gu*H_*alpha;
        
        % simulate the system
        for k = 1:Nw
            [tTemp,xTemp] = ode45(@(t,x)obj.dynamics(x,u_,w(:,k)),[0 timeStep],x_);

            x_ = xTemp(end,:)';
            x = [x;xTemp]; u = [u;ones(size(xTemp,1),1)*u_'];
            if isempty(t)
                t = [t;tTemp];
            else
                t = [t;t(end)+tTemp];
            end
        end
    end
end

function R = reachSetPredSafetyNet(obj,x0,alpha,iter)
% compute reachable set to predict the state at the end of the allocated
% computation time

    % initialization
    U = obj.Usub{iter};
    H = obj.H{iter};
    nu = dim(U{1});
    R0 = zonotope(x0);
    if ~isempty(obj.V)
       R0 = R0 + obj.V; 
    end
    
    % reachability settings
    params.U = zonotope(obj.W);
    params.tFinal = obj.tFinal/(obj.N*obj.Ninter);
    
    options.zonotopeOrder = 10;
    options.taylorTerms = 4;
    options.alg = 'lin';
    options.tensorOrder = 2;
    options.timeStep = params.tFinal;

    % loop over all intermediate time steps
    for i = obj.Npred:obj.Ninter
       
        % compute control input
        cu = center(U{i});
        Gu = generators(U{i});
        
        ind = (obj.Ninter-i)*nu;
        H_ = H(ind+1:ind+nu,:);
        
        u_ = cu + Gu*H_*alpha;
        
        % compute reachable set
        params.R0 = cartProd(R0,u_);
        Rtemp = reachNonlinear(obj.sysPred,params,options);
        R0 = project(Rtemp.timePoint.set{end},1:obj.nx);
    end
    
    R = R0;
end

function alpha = getFactorValues(obj,x0,iter)
% compute the values for the factors alpha by solving the linear program
%
%   min \sum_i |\alpha_i|   s.t. c+G*\alpha = x0, -1 < \alpha_i < 1

    % get zonotope properties
    c = center(obj.reachSet{iter});
    G = generators(obj.reachSet{iter});

    % construct optimization problem
    alpha = sdpvar(size(G,2),1);

    cost = 0;

    for i = 1:size(G,2)
        cost = cost + abs(alpha(i));
    end

    constraints = c + G * alpha == x0;
    constraints = [constraints; -ones(size(G,2),1) <= alpha];
    constraints = [constraints; alpha <= ones(size(G,2),1)];

    % solve linear program with the YALMIP toolbox
    optimize(constraints,cost,sdpsettings('verbose',0));
    
    % store the solution
    alpha = value(alpha);
end

function initSafetyNet(obj)
% initialize reachable set computation to generate all required tensors

    % reachability settings
    params.U = zonotope(obj.U);
    params.tFinal = obj.tFinal/(obj.N*obj.Ninter);
    params.R0 = zonotope([center(obj.Rinit);zeros(obj.nu,1)]);
    
    options.zonotopeOrder = 10;
    options.taylorTerms = 4;
    options.alg = 'lin';
    options.tensorOrder = 2;
    options.timeStep = params.tFinal;
    
    % compute reachable set
    try
       reach(obj.sysPred,params,options); 
    catch  
    end
end
