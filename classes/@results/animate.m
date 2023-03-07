function im = animate(obj,benchmark,varargin)
% ANIMATE - animate the simulated trajectories
%
% Syntax:
%       ANIMATE(obj,benchmark)
%       ANIMATE(obj,benchmark,statObs,dynObs,goalSet)
%       ANIMATE(obj,benchmark,statObs,dynObs,goalSet,speedUp)
%       ANIMATE(obj,benchmark,statObs,dynObs,goalSet,speedUp,addArg)
%
% Description:
%       Animate the simulated trajectories of the controlled system stored 
%       in the results object for the given benchmark.
%
% Input Arguments:
%
%       -obj:           object of class results storing the simulated
%                       trajectories
%       -benchmark:     name of the considered benchmark model (see
%                       "aroc/benchmarks/...")
%       -statObs:       cell-array storing the static obstacles
%       -dynObs:        cell-array storing the dynamic obstacles. Each
%                       dynamic obstacle is represented as a struct with 
%                       fields .set and .time 
%       -goalSet:       goal set for the motion planning problem
%       -speedUp:       speed-up factor for animation
%       -addArg:        additional arguments that are specific for the
%                       considered benchmark (e.g. lanelets for 'car'
%                       benchmark, see "aroc/benchmarks/animation)
%
% See Also:
%       results
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
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------  

    % get simulated trajectories
    if isempty(obj.simulation)
        error('No simulation available in the "results" object!');
    end

    x = obj.simulation{1}.x;
    t = obj.simulation{1}.t;

    % determine animation function for benchmark
    fun = ['animate',upper(benchmark(1)),benchmark(2:end)];
   
    if isempty(which(fun))
        error('No animation function available for this benchmark!');
    end

    % adapt time if speed-up is desired
    if nargin > 5

        speedUp = varargin{4};
        varargin = [varargin(1:3),varargin(5:end)];

        if ~isempty(speedUp)
            t = t/speedUp;
            if ~isempty(varargin{2})
                for i = 1:length(varargin{2})
                    varargin{2}{i}.time = varargin{2}{i}.time/speedUp;
                end
            end
        end
    end

    % animate the system
    if nargin > 2
        if nargout > 0
            eval(['im = ',fun,'(x,t,varargin{:});']);
        else
            eval([fun,'(x,t,varargin{:});']);
        end
    else
        if nargout > 0
            eval(['im = ',fun,'(x,t);']);
        else
            eval([fun,'(x,t);']);
        end
    end
end