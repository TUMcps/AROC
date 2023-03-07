function [U,xCenter,uCenter] = subsetsInput(xc,uc,Opts)
% SUBSETSINPUT - compute sets of admissible inputs for each time step
%
% Syntax:
%       [U,xCenter,uCenter] = SUBSETSINPUT(xc,uc,Opts)
%
% Description:
%       Computes the sets of admissible control inputs due to the shift by
%       the reference trajectory control input for all time steps.
%
% Input Arguments:
%
%       -xc:    reference trajectory states (dimension: [nx,N*Ninter])
%       -uc:    reference trajectory inputs (dimension: [nu,N*Ninter])
%       -Opts:  a structure containing the following options
%
%           -.N:            number of time-steps
%           -.Ninter:       number of intermediate time steps
%           -.uMaxTotal:    maximum allowed control input
%           -.uMinTotal:    minimum allowed control input
%
% Output Arguments:
%
%       -U:         cell-array containing the sets of admissble control
%                   inputs for each time step
%       -xCenter:   cell-array containing the reference trajectory states
%                   for each time step
%       -uCenter:   cell-array containing the reference trajectory inputs
%                   for each time step
%
% See Also:
%       safetyNetControl
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
% Authors:      Moritz Klischat, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialize variables
    xCenter = cell(Opts.N,1);
    uCenter = cell(Opts.N,1);
    U = cell(Opts.N,1);
    counter = 1;

    % loop over all time steps
    for i = 1:Opts.N

       % store center trajectory in the correct format
       xCenter{i} = xc(:,counter:counter + Opts.Ninter);
       uCenter{i} = uc(:,counter:counter + Opts.Ninter-1);
       counter = counter + Opts.Ninter;

       % compute the subsets of the input set due to the center trajectory
       U{i} = cell(Opts.Ninter,1);

       for j = 1:Opts.Ninter   
           uRef = uCenter{i}(:,j);
           width = min([Opts.uMaxTotal-uRef,uRef-Opts.uMinTotal],[],2);
           U{i}{j} = zonotope(interval(uRef-width,uRef+width));
       end  
    end