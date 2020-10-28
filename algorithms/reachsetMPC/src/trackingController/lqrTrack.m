function K = lqrTrack(x0,xf,u,Opts)
% LQRTRACK - compute an LQR controller
%
% Syntax:
%       K = LQRTRACK(x0,xf,u,param)
%
% Description:
%       This function computes the feedback matrix for the tracking
%       controller by applying the LQR control approach to the linearized
%       system
%
% Input Arguments:  
%
%       -x0:    initial state of the considered reference trajectory part
%               (dimension: [nx,1])
%       -xf:    final state of the considered reference trajectory part
%               (dimension: [nx,1])
%       -u:     reference control input for the considered reference 
%               trajectory part (dimension: [nu,1])
%       -Opts:  a structure containing following options
%
%           -.A:    function handle to the system matrix of the linearized
%                   system
%           -.B:    function handle to the input matrix of the linearized
%                   system
%           -.Q:    state weighting matrix for LQR control 
%                   (dimension: [nx,nx])
%           -.R:    input weighting matrix for LQR control
%                   (dimension: [nu,nu])
%
% Output Arguments:
%
%       -K:     computed feedback matrix for the tracking controller 
%               (dimension: [nu,nx])
%
% See Also:
%       reachsetMPC
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

    % determine linearization point
    xLin = (xf+x0)*0.5;

    % linearize the system dynamics
    A = Opts.A(xLin,u);
    B = Opts.B(xLin,u);
    
    % compute the LQR controller
    [k,~,~] = lqr(A,B,Opts.Qlqr,Opts.Rlqr);
    K = -k';
end