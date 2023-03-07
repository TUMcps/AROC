function init(obj)
% INIT - initialize the controller object
%
% Syntax:
%       INIT(obj)
%
% Description:
%       Initializes the controller object before it is used in online
%       control. This function can for example be used to precalculate
%       variables to save computation time.
%
% Input Arguments:
%
%       -obj:   object of class comfContrMPC storing the control law
%               computed in the offline-phase
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

    % Call CORA reach function to initialize the tensors for the system
    % dynamics. We do this only once since this saves a lot of computation
    % time

    % initialization
    R0 = zonotope(obj.xCenter{1}(:,1));
    options = obj.ReachOpts;
    params = obj.ReachParams;
       
    % construct initial set
    params.R0 = cartProd(R0,zonotope(obj.uCenter{1}(:,1)));

    % compute the reachable set
    Rtemp = reach(obj.sys,params,options);  
end