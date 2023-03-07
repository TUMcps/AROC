function [res,R,Param] = reachSet(obj,R0,iter)
% REACHSET - computes the reachable set for an LQR comfort controller
%
% Syntax:
%       [res,R,Param] = REACHSET(obj,R0,iter)
%
% Description:
%       Computes the reachable set for one time step of the LQR 
%       (Linear Quadratic Regulator) tracking controller which is used as a 
%       comfort controller during online application of the Safety Net 
%       Controller.
%
% Input Arguments:
%
%       -obj:   object of class comfContrLQR storing the control law
%               computed in the offline-phase
%       -x0:    initial set for reachable set computatoin
%       -iter:  current iteration (=time step) of the Safety Net Controller
%
% Output Arguments:
%
%       -res:   flag specifying if the constraints are satisfied or not
%       -R:     final reachable set (class: zonotope)
%       -Param: control parameters computed during the online phase which
%               are required for the simulation of the controller later on
%
% See Also:
%       safetyNetControl, simulate, comfContrLQR
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

    % initialization
    res = 1;
    R = [];
    Param = [];

    % get values for the current iteration
    xc = obj.xCenter{iter};
    uc = obj.uCenter{iter};
    K = obj.K{iter};

    % construct the initial set
    options = obj.ReachOpts;
    params = obj.ReachParams;
    params.R0 = cartProd(R0,zonotope(xc(:,1)));
    
    % loop over all intermediate time steps
    for i = 1:obj.Ninter
       
        % pass current feedback matrix and reference input as parameters
        temp = reshape(K{i},[obj.nu*obj.nx,1]);
        params.paramInt = [temp;uc(:,i)];
        
        % compute the reachable set
        Rcont = reachNonlinear(obj.sys,params,options);
        
        % check if input constraints are satisfied
        for j = 1:length(Rcont.timeInterval.set)
            
            % compute set of applied control inputs
            Rtemp = Rcont.timeInterval.set{j};

            U_ = uc(:,i) + K{i}' * zonotope(Rtemp.Z(1:obj.nx,:) ...
                                      - Rtemp.Z(obj.nx+1:end,:));
                                          
            % check input constraints
            if ~contains(obj.U,U_)
                res = 0;
                return;
            end
            
            % check state constraints
            if isa(obj.X,'mptPolytope') && ...
               ~contains(obj.X,project(Rtemp,1:obj.nx))
                res = 0;
                return;
            end
        end
        
        % update the initial set
        Rfin = Rcont.timePoint.set{end};
        params.R0 = Rfin;
    end
    
    % assign output values
    R = project(Rfin,1:obj.nx);
end