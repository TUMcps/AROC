function R = reachNonlinear(obj,params,options)
% REACHNONLINEAR - compute reachable set for nonlienar systems
%
% Syntax:
%       R = REACHNONLINEAR(obj,params,options)
%
% Description:
%       Computes the reachable sets of a nonlinear system object. This
%       function is a copy of the CORA reach function, with the difference
%       that we do not check if the tensors have to be generated since this
%       results in problems when reachability analysis is used in
%       optimization with fmincon
%
% Input Arguments:
%
%       -obj:           object containing the system dynamics (class:
%                       nonlinParamSys or nonlinearSys)  
%       -params:        structure containing the reachability parameters
%       -Opts:          structure containing user defined reachability 
%                       settings   
%
% Output Arguments:
%
%       -R:             object storing the reachable set (class: reachSet) 
%
% See Also:
%       optimizationBasedControl
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

    % option preprocessing
    options = params2options(params,options);
    options = checkOptionsReach(obj,options);

    % obtain factors for initial state and input solution time step
    r = options.timeStep;
    for i = 1:(options.taylorTerms+1)  
        options.factor(i) = r^(i)/factorial(i);    
    end

    % time period
    options.t = options.tStart;
    tVec = options.tStart:options.timeStep:options.tFinal;

    % initialize cell-arrays that store the reachable set
    timeInt.set = cell(length(tVec)-1,1);
    timeInt.time = cell(length(tVec)-1,1);
    timePoint.set = cell(length(tVec)-1,1);
    timePoint.time = cell(length(tVec)-1,1);

    % initialize reachable set computations
    [Rnext, options] = initReach(obj, options.R0, options);

    % loop over all reachability steps
    for i = 2:length(tVec)-1
        
        % save reachable set in cell structure
        timeInt.set{i-1} = Rnext.ti; 
        timeInt.time{i-1} = interval(tVec(i-1),tVec(i));
        timePoint.set{i-1} = Rnext.tp;
        timePoint.time{i-1} = tVec(i);

        % increment time and set counter
        t = tVec(i);
        options.t = t;

        % compute next reachable set
        [Rnext,options] = post(obj,Rnext,options);
    end

    % save last reachable set in cell structure
    timeInt.set{end} = Rnext.ti; 
    timeInt.time{end} = interval(tVec(end-1),tVec(end));
    timePoint.set{end} = Rnext.tp; 
    timePoint.time{end} = tVec(end);
    
    % construct reachset object
    timeInt.set = cellfun(@(x) x{1},timeInt.set,'UniformOutput',false);
    timePoint.set = cellfun(@(x) x{1}.set,timePoint.set,'UniformOutput',false);

    R = reachSet(timePoint,timeInt);
end