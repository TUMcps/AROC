function res = mirror(obj,states,inputs,varargin)
% MIRROR - mirror a motion primitive along the given symmetry axis
%
% Syntax:
%       res = MIRROR(obj,states,inputs)
%
% Description:
%       Mirrors a motion primitive along the given symmetry axis for 
%       states, inputs, and occupancy set. The function can for example be 
%       used to construct a equivlalent "turn-left" motion primitive from a
%       "turn-right" motion primitive without running controller synthesis 
%       again. This is especially useful for constructing large maneuver 
%       automata. 
%
% Input Arguments:
%
%       -obj:           object of class comfContrLQR
%       -states:        indices of the states that are mirrored
%       -inputs:        indices of the inputs that are mirrored
%
% Output Arguments:
%
%       -res:   mirrored motion primitive (class comfContrLQR)
%
% See Also:
%       safetyNetControl, comfContrLQR
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

    % construct tranformation matrices for mirroring
    tmp = ones(obj.nx,1); 
    tmp(states) = -tmp(states); 
    Tx = diag(tmp);

    tmp = ones(obj.nu,1);
    tmp(inputs) = -tmp(inputs);
    Tu = diag(tmp);

    % copy original object
    res = obj;

    % mirror feedback matrices
    for i = 1:length(res.K)
        for j = 1:length(res.K{i})
            res.K{i}{j} = (Tu * res.K{i}{j}' * Tx)';
        end
    end

    % mirror reference trajectory
    for i = 1:length(res.uCenter)
        res.uCenter{i} = Tu * res.uCenter{i};
        res.xCenter{i} = Tx * res.xCenter{i};
    end
    
    % mirror final state
    res.xf = Tx * res.xf;

    % mirror input set
    res.U = Tu * res.U;
