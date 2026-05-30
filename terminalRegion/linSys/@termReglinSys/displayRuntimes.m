function displayRuntimes(T)
% DISPLAYRUNTIMES - displays the runtimes of a termRegLinSysContainment
%                   object
%
% Syntax:
%       T.displayRuntimes()
%
% Description:
%       This function displays the runtimes when performing the
%       LinSys-Containment algorithm to compute a terminal region
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

disp("Runtime Summary:")
disp("Preprocessing:   "+num2str(T.runtimes.preprocessing))
disp("Reachability:    "+num2str(T.runtimes.reachability))
disp("SIC Constraints: "+num2str(T.runtimes.state_input_containment_constraints))
disp("Solver:          "+num2str(T.runtimes.solver))
disp("-----------------")
disp("Total Time:      "+num2str(T.runtimes.total))