function R = reachSetPred(obj,x0,iter,Param)
% REACHSETPRED - computes the reachable set for an MPC comfort controller
%
% Syntax:
%       R = REACHSETPred(obj,x0,iter,Param)
%
% Description:
%       Computes the reachable set for an MPC (Model Predictive Control) 
%       comfort controller for the allocated computation time to predict
%       where the system state will be when the synthesis and verification
%       of the comfort controller is finished.
%
% Input Arguments:
%
%       -obj:   object of class comfContrLQR storing the control law
%               computed in the offline-phase
%       -x0:    initial point for the reachable set computation
%       -iter:  current iteration (=time step) of the Safety Net Controller
%       -Param: control parameters computed during the online phase
%
% Output Arguments:
%
%       -R:     final reachable set (class: zonotope)
%
% See Also:
%       safetyNetControl, simulate, comfContrMPC
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
    R0 = zonotope(x0);
    if ~isempty(obj.V)
       R0 = R0 + obj.V; 
    end
    
    options = obj.ReachOpts;
    params = obj.ReachParams;
    
    % loop over all intermediate time steps
    for i = obj.Npred:obj.Ninter
       
        % construct initial set
        params.R0 = cartProd(R0,zonotope(Param(:,i)));
        
        % compute the reachable set
        Rtemp = reachNonlinear(obj.sys,params,options);
        
        % update the initial set
        Rfin = Rtemp.timePoint.set{end};
        R0 = project(Rfin,1:obj.nx);
    end
    
    % assign output values
    R = project(Rfin,1:obj.nx);
end