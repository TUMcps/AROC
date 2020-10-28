function c = checkConstraints(R,K,u_ref,Opts)
% CHECKCONSTRAINTS - check if the input and state constraints are satisfied
%
% Syntax:
%       c = CHECKCONSTRAINTS(R,K,u_ref,Opts)
%
% Description:
%       This function checks if the reachable set satisfies the state and
%       input constraints. The function returns a vector that specifies how 
%       much each constraint is violate, where c < 0 means that all
%       constraints are satisfied.
%
% Input Arguments:
%
%       -R:     cell-array storing the reachable set
%       -K:     feedback matrix for the control law u = u_ref + K(x-x_ref)
%       -u_ref: reference input for the control law u = u_ref + K(x-x_ref)
%       -Opts:  structure containing the following options
%
%           -.stateCon:     structure containing the state constraints
%           -.inputCon:     structure containing the input constraints
%           -.nx:           number of states
%           -.nu:           number of inputs
%
% Output Arguments:
%
%       -c:     amount of violation for the inequality constraints 
%               (c <= 0: no violation)
%
% See Also:
%       optimizationBasedControl, conFun
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
% Authors:      Ivan Hernandez, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    c = [];

    % loop over all reachable sets
    for i = 1:length(R)
            
        % get the current time interval reachable set
        Rtemp = R{i};
        
        % compute set of applied control inputs
        if isa(Rtemp,'polyZonotope')
            
            c = Rtemp.c(1:Opts.nx,:) - Rtemp.c(Opts.nx+1:end,:);
            G = Rtemp.G(1:Opts.nx,:) - Rtemp.G(Opts.nx+1:end,:);
            Grest = Rtemp.Grest(1:Opts.nx,:) - Rtemp.Grest(Opts.nx+1:end,:);
            
            U_ = u_ref + K * polyZonotope(c,G,Grest,Rtemp.expMat);
        else
            U_ = u_ref + K * zonotope(Rtemp.Z(1:Opts.nx,:) ...
                                      - Rtemp.Z(Opts.nx+1:end,:));
        end

        % check input constraints
        temp = supremum(interval(Opts.inputCon.A*U_ + (-Opts.inputCon.b)));
        c = [c;temp];

        % check state constraints
        if ~isempty(Opts.stateCon)
            Rtemp = project(Rtemp,1:Opts.nx);
            temp = supremum(interval(Opts.stateCon.A*Rtemp - Opts.stateCon.b));
            c = [c;temp];
        end
    end       
end