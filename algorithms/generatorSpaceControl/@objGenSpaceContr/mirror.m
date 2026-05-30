function res = mirror(obj,states,inputs,varargin)
% MIRROR - mirror a motion primitive along the given symmetry axis
%
% Syntax:
%       res = MIRROR(obj,states,inputs)
%       res = MIRROR(obj,states,inputs,occupancy)
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
%       -obj:           object of class objGenSpaceContr
%       -states:        indices of the states that are mirrored
%       -inputs:        indices of the inputs that are mirrored
%       -occupancy:     indices of the occupancy set dimensions that are
%                       mirrored
%
% Output Arguments:
%
%       -res:   mirrored motion primitive (class objGenSpaceContr)
%
% See Also:
%       generatorSpaceControl, objGenSpaceContr
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

    % check if system dynamics is symmetric
    if ~checkSymmetry(obj,states,inputs)
        error('System dynamics is not symmetric!')
    end

    % construct tranformation matrices for mirroring
    tmp = ones(obj.nx,1); 
    tmp(states) = -tmp(states); 
    Tx = diag(tmp);

    tmp = ones(obj.nu,1);
    tmp(inputs) = -tmp(inputs);
    Tu = diag(tmp);

    if nargin > 3 && ~isempty(varargin{1}) && ~isempty(obj.occupancySet)
        occupancy = varargin{1};
        tmp = ones(dim(obj.occupancySet{1}.set),1);
        tmp(occupancy) = -tmp(occupancy);
        To = diag(tmp);
    end

    % copy original object
    res = obj;

    % mirror parallelotopes
    for i = 1:length(res.parallelo)
        res.parallelo{i} = Tx * res.parallelo{i};
    end
    
    % mirror final set and state
    res.xf = Tx * res.xf;
    res.Rfin = Tx * res.Rfin;

    % mirror iniatial set and input set
    res.R0 = Tx * res.R0;
    res.U = Tu * res.U;

    % mirror occupancy set
    if ~isempty(obj.occupancySet)
        for i = 1:length(obj.occupancySet)
            res.occupancySet{i}.set = To * res.occupancySet{i}.set;
        end
    end