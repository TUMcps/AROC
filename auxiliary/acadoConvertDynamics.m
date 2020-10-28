function acadoConvertDynamics(path,Opts,benchmark)
% ACADOCONVERTDYNAMICS - Compute a zonotope approximating the inputs
%
% Syntax:
%       ACADOCONVERTDYNAMICS(path,Opts,benchmark)
%
% Description:
%       Convert the dynamic equations describing the system to the format
%       required for the ACADO toolbox for solving optimal control
%       problems. The dynamic equations in the correct format are written
%       to a file.
%
% Input Arguments:
%
%       -path:          path to the root-folder of the Convex Interpolation
%                       Control Algorithm
%       -Opts:          structure containing user defined options for the 
%                       algorithm 
%       -benchmark:     name of the considered benchmark model
%
% See Also:
%       convexInterpolationControl, writeAcadoFiles
%
% References:
%       * *[1] Schuermann et al. (2017)*, Convex interpolation control with 
%              formal guarantees for disturbed and constrained nonlinear 
%              systems
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


    % function handle to dynamic file
    str = ['funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % open file to which the dynamic function in ACADO format is written
    benchPath = fullfile(path,'acado',benchmark);
    if ~exist(benchPath,'dir')
       mkdir(benchPath);
    end
    
    filename = [benchmark,'_acado.m'];
    filePath = fullfile(benchPath,filename);
    fid = fopen(filePath,'w');
    
    % express the system dynamics as a symbolic function
    x = sym('X',[Opts.nx,1]);
    u = sym('U',[Opts.nu,1]);
    w = sym(zeros(Opts.nw,1));
    t = sym(0);
    
    f = funHandle(x,u,w);
    
    % convert symbolic function to string
    for  i = 1:length(f)
        
       % convert to string
       str = char(f(i)); 
       
       % replace state variables
       for j = 1:Opts.nx
          oldStr = sprintf('X%i',j);
          newStr = sprintf('$x%i$',j);
          str = strrep(str,oldStr,newStr);
       end
       
       % replace input variables
       for j = 1:Opts.nu
          oldStr = sprintf('U%i',j);
          newStr = sprintf('$u%i$',j);
          str = strrep(str,oldStr,newStr);
       end
       
       % convert to acado format
       strTemp = sprintf('dot($x%i$)',i);
       str = ['f.add(',strTemp,' == ',str,');'];
       
       % write to file
       fprintf(fid,'%s\n', str);
       
    end
    
    % close file
    fclose(fid);
end