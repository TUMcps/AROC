function res = checkSymmetry(obj,states,inputs)
% CHECKSYMMETRY - check symmetry of the system dynamics
%
% Syntax:
%       res = CHECKSYMMETRY(obj,states,inputs)
%
% Description:
%       Checks if the system dynamics is symmetric with respect to the 
%       given symmetry-axes for states and inputs. 
%
% Input Arguments:
%
%       -obj:           object of class objController
%       -states:        indices of the states that are mirrored
%       -inputs:        indices of the inputs that are mirrored
%
% Output Arguments:
%
%       -res:   symmetric (res = true) or not (res = false)
%
% See Also:
%       objController, mirror
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

    % generate symbolic system dynamics
    x = sym('x',[obj.nx,1]);
    u = sym('u',[obj.nu,1]);
    w = sym('w',[obj.nw,1]);

    f = obj.dynamics(x,u,w);

    % generate mirrored system dynamics
    x_ = x; x_(states) = -x(states);
    u_ = u; u_(inputs) = -u(inputs);

    f1 = obj.dynamics(x_,u_,w);

    % also mirror uncertainties
    if isequal(obj.W,-obj.W)
        f2 = obj.dynamics(x_,u_,-w);
    else
        f2 = f1;
    end

    % check correcness
    res = true;

    for i = 1:obj.nx
        if ismember(i,states)
            if ~isequal(f(i),-f1(i)) && ~isequal(f(i),-f2(i))
                res = false; return;
            end
        else
            if ~isequal(f(i),f1(i)) && ~isequal(f(i),f2(i))
                res = false; return;
            end
        end
    end
end