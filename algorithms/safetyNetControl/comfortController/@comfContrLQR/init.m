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
%       -obj:   object of class comfContrLQR storing the control law
%               computed in the offline-phase
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

    % Call CORA reach function to initialize the tensors for the system
    % dynamics. We do this only once since this saves a lot of computation
    % time

    % get reference trajectory + feedback matrix
    xc = obj.xCenter{1};
    uc = obj.uCenter{1};
    K = obj.K{1};

    % construct the initial set
    options = obj.ReachOpts;
    params = obj.ReachParams;
    params.R0 = zonotope([xc(:,1);xc(:,1)]);
       
    % pass current feedback matrix and reference input as parameters
    temp = reshape(K{1},[obj.nu*obj.nx,1]);
    params.paramInt = [temp;uc(:,1)];
        
    % compute the reachable set
    Rcont = reach(obj.sys,params,options);
        
end