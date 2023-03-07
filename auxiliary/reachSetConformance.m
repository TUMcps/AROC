function R = reachSetConformance(benchmark,measurement,W,varargin)
% REACHSETCONFORMANCE - compute reachable set of a conformant model
%
% Syntax:
%       R = REACHSETCONFORMANCE(benchmark,measurement,W)
%       R = REACHSETCONFORMANCE(benchmark,measurement,W,V)
%
% Description:
%       This function computes the reachable set of a conformant system 
%       model for the input commands corresponding to the given 
%       measurement.
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -measurement:  measurement represented as a struct with the 
%                      following fields: 
%
%           -.x:       matrix storing the measured trajectory of the real
%                      system (dimension: [n,N])
%           -.u:       matrix storing the input signal corresponding to the
%                      measured trajectory (dimension: [m,N-1])
%           -.t:       vector storing the time points for the measured
%                      trajectory (dimension: [1,N])
%
%       -W:            set of disturbances
%       -V:            set of measurement errors
%
% Output Arguments:
%
%       -R:            reachable set (class: reachSet)
%
% See Also:
%       conformantSynthesis, conformantSynthesisNonlinear
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

    % initialization
    V = [];
    if nargin > 3
        V = varargin{1};
    end
    measurement.t = measurement.t - measurement.t(1);
    nu = size(measurement.u,1);
    nw = dim(W);

    % define nonlinear system object
    name = ['conformace', benchmark]; 
    str = ['funHan = @(x,u)',benchmark,'(x,u(1:nu),u(nu+1:end));'];
    eval(str);

    sys = nonlinearSys(name, funHan);

    % algorithm settings
    options.alg = 'lin';
    options.tensorOrder = 2;
    options.zonotopeOrder = 10;
    options.taylorTerms = 5;
    options.timeStep = measurement.t(end) / size(measurement.u,2);

    % reachability parameter
    params.R0 = zonotope(measurement.x(:,1));
    params.U = zonotope(cartProd(zeros(nu,1),W));
    params.u = [measurement.u; zeros(nw,size(measurement.u,2))];
    params.tFinal = measurement.t(end);

    % compute reachable set
    R = reach(sys,params,options);

    % add measurement error
    if ~isempty(V)
        R = R + zonotope(V);
    end
end