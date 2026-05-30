classdef terminalRegion
% TERMINALREGION - abstract class for all terminal regions in AROC
%
% Syntax:
%       obj = TERMINALREGION(dyn,set,Param)
%
% Description:
%       This class is the superclass for all terminal region objects in the 
%       AROC toolbox. The class defines some common properties that are
%       inherited by all terminal region objects.
%
% Input Arguments:
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -set:       set representing the terminal region (class: contSet) 
%       -Param:     a structure containing the benchmark parameters
%
%           -.U:        set of admissible control inputs 
%                       (class: interval or zonotope)
%           -.W:        set of uncertain disturbances 
%                       (class: interval or zonotope)
%           -.V:        set of measurement errors 
%                       (class: interval or zonotope)
%           -.X:        set of state constraints (class: polytope)
%
% Output Arguments:
%
%       -obj:   generated object of class terminalRegion
%
% See Also:
%       termRegSubpaving, termRegZonoLinSys, termRegNonlinSysOpt
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
% Authors:      Niklas Kochdumper, Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    dynamics = [];      % function handle to the dynamic function
    set = [];           % set representing the terminal region
    U = [];             % set of admissible control inputs
    W = [];             % set of disturbances
    V = [];             % set of measurement errors
    X = [];             % set of state constraints

    lp_poblem struct = struct() % struct storing the parametersof the LP returning the parameterization of a given state in the terminal region
end
   
methods
    
    function obj = terminalRegion(dyn,set,Param)
    % class constructor   
    
        obj.dynamics = dyn;
        obj.set = set;
        obj.U = Param.U;
        obj.W = Param.W;
        
        if isfield(Param,'V')
            obj.V = Param.V;
        else
            obj.V = []; 
        end
        
        if isfield(Param,'X')
            obj.X = Param.X;
        else
            obj.X = [];
        end

        % compute coefficients of linear program to obtain the 
        % parameterization of a given state in the terminal region
        if isa(set,'zonotope') || ...
                (isa(set,'polyZonotope') && representsa(set,'zonotope'))
            nGen = size(set.G,2);
            nx = length(set.c);
            problem = struct();
            problem.f = [1; zeros(nGen,1)];
            problem.Aineq = ...
                [-ones(nGen,1), eye(nGen); ...
                -ones(nGen,1), -eye(nGen)];
            problem.bineq = zeros(2*nGen,1);
            problem.Aeq = [zeros(nx,1), set.G];
            problem.beq = -set.c;
            problem.lb = [];
            problem.ub = [];
            obj.lp_poblem = problem;
        end
    end  
end

methods (Access = public)
    function lambda = computeStateParameterization(obj,x)
        % Computes the parameterization of the state x in the terminal
        % region, i.e., find a vector lambda so that
        %   x = obj.set.c + obj.set.G*lambda, |lambda| <= 1
        % by solving a linear program.
        %
        % Input Argument:
        %       -x:     current state of the dynamical system
        %               (double vector)
        %
        % Output Argument:
        %       -lambda:    parameterization of x 
        %                   (double vector)
        %
        % ------------------------------------------------------------
        
        % compute right-hand side of equality constraint
        tmp_problem = obj.lp_poblem;
        tmp_problem.beq = x + tmp_problem.beq;

        % solve linear program
        yOpt = CORAlinprog(tmp_problem);
        lambda = yOpt(2:end);
                    tol = 1e-6;
        if yOpt(1) > 1 + tol
            warning('(Measured) State is not contained in the terminal region!')
        end    
    end
end
end