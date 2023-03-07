function R = reachSetPred(obj,x0,iter,Param)
% REACHSETPRED - computes the reachable set for an LQR comfort controller
%
% Syntax:
%       R = REACHSETPRED(obj,x0,iter,Param)
%
% Description:
%       Computes the reachable set for an LQR (Linear Quadratic Regulator)
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

    % get values for the current iteration
    xc = obj.xCenter{iter};
    uc = obj.uCenter{iter};
    K = obj.K{iter};

    % construct the initial set
    options = obj.ReachOpts;
    params = obj.ReachParams;
    params.R0 = zonotope([x0;xc(:,3)]);
    
    if ~isempty(obj.V)
       params.R0 = params.R0 + cartProd(obj.V,zeros(obj.nx,1)); 
    end
    
    % loop over all intermediate time steps
    for i = obj.Npred:obj.Ninter
       
        % pass current feedback matrix and reference input as parameters
        temp = reshape(K{i},[obj.nu*obj.nx,1]);
        params.paramInt = [temp;uc(:,i)];
        
        % compute the reachable set
        Rcont = reachNonlinear(obj.sys,params,options);
        
        % update the initial set
        Rfin = Rcont.timePoint.set{end};
        params.R0 = Rfin;
    end
    
    % assign output values
    R = project(Rfin,1:obj.nx);
end