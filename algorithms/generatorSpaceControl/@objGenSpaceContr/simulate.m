function [res,t,x,u] = simulate(obj,res,x0,w)
% SIMULATE - simulate a trajectory of a nonlinear system controlled with 
%            the controller based on optimal control in generator space
%
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,res,x0,w)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with the controller based on optimal control in generator space for 
%       a given initial point and given specific disturbance values over 
%       time.
%
% Input Arguments:
%
%       -obj:   object of class objGenSpaceContr storing the control law
%               computed in the offline-phase
%       -res:   existing results object to which the simulation results
%               should be added
%       -x0:    initial point for the simulation
%       -w:     matrix storing the values for the disturbances over time
%               (dimension: [nw,N*Ninter])
%
% Output Arguments:
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the control inputs for the simulated
%               trajectory (dimension: [|t|,nu])
%
% See Also:
%       generatorSpaceContr, simulateRandom
%
% References:
%       * *[1] Schuermann et al. (2017)*, Guaranteeing constraints of 
%              disturbed nonlinear systems using set-based optimal 
%              control in generator space
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
% Authors:      Jan Wagener, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

    % check if the number of specified disturbance vectors is correct
    Nw = 1;
    
    if size(w,2) > 1
        Nw = size(w,2)/(obj.N * obj.Ninter);
        if Nw < 1 || floor(Nw) ~= Nw
           error('Number of disturbance vectors has to be a multiple of ''Nc*Ninter''!'); 
        end
    end

    % time step size
    dt = obj.tFinal/(obj.N*obj.Ninter*Nw);
    
    % dynamic function and input set parameters
    f = obj.dynamics;
    
    cu = center(obj.U);
    Gu = generators(obj.U);
    
    % initialization
    x_ = x0;
    counter = 1;
    x = []; t = []; u = [];

    % loop over all time steps
    for i = 1:obj.N

        % compute factors beta for the current point
        cx = center(obj.parallelo{i});
        Gx = generators(obj.parallelo{i});
        beta = Gx\(x_ - cx);

        % loop over all intermediate time steps
        for j = 1:obj.Ninter

            % compute current control law (see Eq. (15) in [1])
            alpha = obj.alpha{i}{j};
            alpha_c = alpha(:,1);
            alpha_g = alpha(:,2:end);

            u_ = cu + Gu * (alpha_c + alpha_g * beta);

            % loop over all disturbances
            for k = 1:Nw
                
                % simulate the system
                [tTemp,xTemp] = ode45(@(t,x) f(x,u_,w(:,counter)),[0 dt],x_);
                
                % update initial state
                x_ = xTemp(end,:)';
                counter = counter + 1;
                
                % store the results
                x = [x;xTemp]; u = [u;ones(size(xTemp,1),1)*u_'];
                if isempty(t)
                    t = [t;tTemp];
                else
                    t = [t;tTemp + t(end)]; 
                end
            end
        end
    end

    % store simulation in results object
    if isempty(res)
        sim{1}.t = t; sim{1}.x = x; sim{1}.u = u; sim{1}.flag = flag;
        res = results([],[],[],sim);
    else
        sim = res.simulation;

        if isempty(sim)
           sim{1}.t = t; sim{1}.x = x; sim{1}.u = u; sim{1}.flag = flag;
        else
           sim{end+1}.t = t; sim{end}.x = x; 
           sim{end}.u = u; sim{end}.flag = flag;
        end

        res = results(res.reachSet,res.reachSetTimePoint,res.refTraj,sim);
    end
end