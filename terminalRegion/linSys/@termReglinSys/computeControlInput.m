function u = computeControlInput(T, y, nTimeStep, beta, beta_V)
% COMPUTECONTROLINPUT - computes the control inputs one has to choose so
%                       that an output y starting in the terminal region
%                       ends up again in the terminal region
%
% Syntax:
%        u = computeControlInput(T, y, currentInput, nTimeStep, beta)
%        u = computeControlInput(T, y, currentInput, nTimeStep, beta, beta_V)
%
% Input Arguments:
%       -y:          A point that lies in the terminal region of the output
%                    (and for which the state lies in the state terminal
%                    region) if nTimeStep=1, and lies in the reachable set
%                    of the terminal region at time
%                    (nTimeStep-1) * T.Opts.timeStep
%                    (class: double)
%
%       -nTimeStep:  Index of the current time step (class: double, must be
%                    a positive integer)
%
%       -beta,beta_V:Initial parametrization of y0; in other words, if y is
%                    the current trajectory that started with y0, then beta
%                    is given by
%                    y0 = G_0 * beta + c_0 + G_V * beta_V
%                    where G_0 and c_0 are the generator matrix and center
%                    of the terminal region, G_V is the generator matrix of
%                    the noise set.
%
% Output Arguments:
%       -u:       Inputs that have to be applied at time
%                 nTimeStep * T.Opts.timeStep, in order to continue
%                 steering the system back to the terminal region
%
% Description:
%       Computes the control inputs one has to choose so that an output y
%       starting in the terminal region ends up again in the terminal
%       region.
%
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
% Authors:      Adrian Kulmburg
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

if CHECKS_ENABLED

    inputArgsCheck({{y,'att','numeric',{'vector','nonnan','finite'}}});
    equalDimCheck(T.set, y);
    inputArgsCheck({{nTimeStep,'att','numeric',{'scalar','nonnan','finite','positive','integer'}}});

    if nTimeStep > T.Opts.N
        error('nTimeStep must be a positive integer between 1 and T.Opts.N.')
    end

end

termReg = T.set;
errorSet = T.V;

if CHECKS_ENABLED
    inputArgsCheck({{beta,'att','numeric',{'vector','nonnan','finite'}}});
    inputArgsCheck({{beta,'att','numeric',{'vector','nonnan','finite'}}});
    
    if strcmp(T.Opts.terminalRegionType, 'zonotope') && size(generators(termReg),2) ~= length(beta)
        error("Dimension mismatch between alpha and the number of generators for the (error-free) initial set.")
    elseif strcmp(T.Opts.terminalRegionType, 'ellipsoid') && dim(termReg) ~= length(beta)
        error("Dimension mismatch between alpha and the number of generators for the (error-free) initial set.")
    elseif size(generators(errorSet),2) ~= length(beta_V)
        error("Dimension mismatch between beta and the number of generators for the measurements error set.")
    end
end


K = T.feedback_speedup.K;

U_X = T.termRegCtrl{nTimeStep}.U_X;
U_V = T.termRegCtrl{nTimeStep}.U_V;
u_c = T.termRegCtrl{nTimeStep}.u_c;

% We can now compute the control input quite easily
u = K * y + U_X * beta + U_V * beta_V + u_c;

end