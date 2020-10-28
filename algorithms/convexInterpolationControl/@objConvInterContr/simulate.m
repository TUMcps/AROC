function [res,t,x,u] = simulate(obj,res,x0,w)
% SIMULATE - simulate a trajectory of a nonlinear system controlled with 
%            the Convex Interpolation Controller
%
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,res,x0,w)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with the Convex Interpolation Controller for a given initial point
%       and given specific disturbance values over time.
%
% Input Arguments:
%
%       -obj:   object of class objConvInterContr storing the control law
%               computed in the offline-phase
%       -res:   existing results object to which the simulation results
%               should be added
%       -x0:    initial point for the simulation
%       -w:     matrix storing the values for the disturbances over time
%               (dimension: [nw,timeSteps])
%
% Output Arguments:
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the applied control input 
%               (dimension: [|t|,nu])
%
% See Also:
%       convexInterpolationControl, simulateRandom
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
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    


    % check if the number of specified disturbance vectors is correct
    Nw = 1;
    
    if size(w,2) > 1
        Nw = size(w,2)/(obj.Nc * obj.Ninter);
        if Nw < 1 || floor(Nw) ~= Nw
           error('Number of disturbance vectors has to be a multiple of ''Nc*Ninter''!'); 
        end
    end

    % compute time step 
    timeStep = (obj.tFinal/obj.Nc)/(obj.Ninter);
    
    % simulate each part with constant input
    x_ = x0;
    xu = x0;
    x = []; u = []; t = [];
    counter = 1;
    
    for i = 1:obj.Nc
        for j = 1:obj.Ninter
            
            % compute control input
            if strcmp(obj.controller,'linear')
                u_ = obj.controlLawParam{i}.A{j} * xu + obj.controlLawParam{i}.b{j};
            elseif strcmp(obj.controller,'quadratic')
                A = obj.controlLawParam{i}.A{j};
                b = obj.controlLawParam{i}.b{j};
                o = obj.controlLawParam{i}.o{j};
                u_ = zeros(obj.nu,1);
                for k = 1:obj.nu
                   u_(k) = xu'*A{k}*xu + b{k}'*xu + o{k}; 
                end
            else
                u_ = exactControlLaw(xu,obj.parallelo{i},obj.controlLawParam{i}{j});
            end
            
            % simulate the system
            for k = 1:Nw
                [tTemp,xTemp] = ode45(@(t,x)obj.dynamics(x,u_,w(:,counter)),[0 timeStep/Nw],x_);

                x_ = xTemp(end,:)';
                x = [x;xTemp];
                u = [u;ones(size(xTemp,1),1)*u_'];
                if isempty(t)
                    t = [t;tTemp];
                else
                    t = [t;t(end)+tTemp];
                end
                counter = counter + 1;
            end
        end
        
        xu = x_;
    end

    % store simulation in results object
    if isempty(res)
        simulation{1}.t = t;
        simulation{1}.x = x;
        simulation{1}.u = u;
        res = results([],[],[],simulation);
    else
        simulation = res.simulation;

        if isempty(simulation)
           simulation{1}.t = t;
           simulation{1}.x = x;
           simulation{1}.u = u;
        else
           simulation{end+1}.t = t;
           simulation{end}.x = x; 
           simulation{end}.u = u;
        end

        res = results(res.reachSet,res.reachSetTimePoint,res.refTraj,simulation);
    end
end