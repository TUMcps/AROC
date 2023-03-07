function Opts = symLinearize(Opts)
% SYMLINEARIZE - linearize System and save in Opts
%
% Syntax:
%       Opts = SYMLINEARIZE(Opts);
%
% Description:
%       linearize System and save in Opts
%
%
% Input Arguments:
%
%       -Opts:      Options structure
%
% Output Arguments:
%
%       -Opts:      Options structure
%
% See Also:
%       -
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
% Authors:      Jan Wagener, Niklas Kochdumper, Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % define symbolic variables
    x = sym('x',[Opts.nx,1]); 
    u = sym('u',[Opts.nu,1]);
    w = sym('w',[Opts.nw,1]);
    
    % get symbolic expression for the system dynamics
    f = Opts.funHandle(x,u,w);
    
    % compute derivatives
    A = jacobian(f,x);
    B = jacobian(f,u);
    Bw = jacobian(f,w);

    % generate function handles for linearized dynamics
    Opts.linDyn.A = matlabFunction(A,'Vars',{x,u,w});
    Opts.linDyn.B = matlabFunction(B,'Vars',{x,u,w});
    Opts.linDyn.Bw = matlabFunction(Bw,'Vars',{x,u,w});
end