function [W,V] = conformantSynthesis(benchmark,measurements,varargin)
% CONFORMANTSYNTHESIS - compute model uncertainty to enclose measurements
%
% Syntax:
%       [W,V] = CONFORMANTSYNTHESIS(benchmark,measurements)
%       [W,V] = CONFORMANTSYNTHESIS(benchmark,measurements,Opts)
%
% Description:
%       This function computes the model uncertainty that is required such 
%       that the uncertain model encloses all measurements of the real 
%       system. The model uncertainty consists of a set of uncertain inputs
%       W and a set of measurement errors V.
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -measurements: cell-array storing the measurements of the real
%                      system, where each cell is a struct with the 
%                      following fields: 
%
%           -.x:       matrix storing the measured trajectory of the real
%                      system (dimension: [n,N])
%           -.u:       matrix storing the input signal corresponding to the
%                      measured trajectory (dimension: [m,N-1])
%           -.t:       vector storing the time points for the measured
%                      trajectory (dimension: [1,N])
%
%       -Opts:         a structure containing the algorithm settings
%
%           -.set:     string specifying the type of set which is used to
%                      represent the uncertainty.
%                      [{'interval'} / 'zonotope']
%           -.group:   number of measurements grouped together
%                      [{1} / positive integer]
%           -.measErr: use measurement error to capture uncertainty
%                      [{false} / true]
%           -.mu       tradeoff between uncertain inputs and measurement
%                      errors
%                      [{0.5} / double between 0 and 1]
%
% Output Arguments:
%
%       -W:     resulting set of uncertain inputs
%       -V:     resulting set of measurement errors
%
% See Also:
%       convexInterpolationControl, generatorSpaceControl
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
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % parse input arguments
    if nargin > 2
        Opts = varargin{1};
    else
        Opts = [];
    end

    Opts = checkOpts(Opts,mfilename);
    
    % function handle to the dynamic file
    str = ['funHan = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    [count,out] = inputArgsLength(funHan,3);

    Opts.nx = out;
    Opts.nu = count(2);
    Opts.nw = count(3);

    % check format of measurements
    for i = 1:length(measurements)
        fields = {'x','u','t'};
        for j = 1:length(fields)
            if ~isfield(measurements{i},fields{j})
                str = ['Wrong format for measurements! Field .', ...
                                         fields{j}, ' is missing!'];
                error(str);
            end
        end
        if size(measurements{i}.x,1) ~= Opts.nx
            str = ['Wrong dimension for field .x of measurement ', ...
                                                    num2str(i), '!'];
            error(str);
        end
        if size(measurements{i}.u,1) ~= Opts.nu
            str = ['Wrong dimension for field .u of measurement ', ...
                                                     num2str(i), '!'];
            error(str);
        end
        if size(measurements{i}.x,2) ~= size(measurements{i}.u,2)+1 || ...
                size(measurements{i}.x,2) ~= length(measurements{i}.t)
            str = ['Dimensions for measurement ', num2str(i), ...
                                                'are not consistent!'];
            error(str);
        end
    end

    % check if model is linear or nonlinear
    [res,A,B,D,c] = isLinearModel(funHan,Opts.nx,Opts.nu,Opts.nw);

    if res
        [W,V] = conformantSynthesisLinear(A,B,D,c,measurements,Opts);
    else
        [W,V] = conformantSynthesisNonlinear(benchmark,measurements,Opts);
    end
end
