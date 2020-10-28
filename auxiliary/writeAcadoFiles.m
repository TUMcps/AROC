function writeAcadoFiles(path,benchmark,Opts)
% WRITEACADOFILES - Generate files for ACADO Toolbox
%
% Syntax:
%       WRITEACADOFILES(path,benchmark,Opts)
%
% Description:
%       This function generates and compiles the files containing the
%       system dynamics, which are needed for solving optimal control
%       problems with the ACADO toolbox
%
% Input Arguments:
%
%       -path:      path of the root directory for the Convex Interpolation
%                   Control Algorithm
%       -benchmark: name of the selected benchmark
%       -Opts:              a structure containing following options
%
%           -.nx:                   number of system states
%           -.nu:                   number of system inputs
%           -.extHorizon.active:    use extended optimization horizon for 
%                                   optimal control problems (optional)
%                                   [{false} / true]
%           -.extHorizon.horizon:   length of the extended optimization
%                                   horizon in center trajectory time steps
%                                   (optional) 
%                                   [{'all'} / positive integer]
%           -.extHorizon.decay:     decay function for the objective
%                                   function of the optimization problem
%                                   with extended optimization horizon
%                                   (optional)
%                                   [{'uniform'} / 'fall' / 'fall+end' / 
%                                    'fallLinear' / 'fallLinear+End' / 
%                                    'fallEqDiff' / 'FallEqDiff+End' / 
%                                    'rise' / 'quad' /  'riseLinear' /
%                                    'riseEqDiff' / 'end']
%
% See Also:
%       acadoConvertDynamics
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

    % Parse options
    extHorizon.active = false;
    extHorizon.decay = 'uniform';
    extHorizon.horizon = 'all';
    
    if ~isfield(Opts,'extHorizon')
       Opts.extHorizon = extHorizon;
    else
        if ~isfield(Opts.extHorizon,'active')
           Opts.extHorizon.active = false; 
        end
        if ~isfield(Opts.extHorizon,'decay')
           Opts.extHorizon.decay = 'uniform'; 
        end
        if ~isfield(Opts.extHorizon,'horizon')
           Opts.extHorizon.horizon = 'all'; 
        end
    end

    % Read in the system dynamics
    filename = [benchmark,'_acado.m'];
    filePath = fullfile(path,'acado',benchmark,filename);
    text = fileread(filePath);
        
    % Check if the system dynamics got changed
    generate = 0;
    filenameOld = [benchmark,'_acado_lastVersion.m'];
    filePathLast = fullfile(path,'acado',benchmark,filenameOld);
    
    if ~exist(filePathLast,'file')
       generate = 1; 
    else
       textOld = fileread(filePathLast);
       textNew = fileread(filePath);
       if ~strcmp(textOld,textNew)
           generate = 1;
       end
    end
    
    % Delete all old files if the dynamics got changed
    if generate
       movefile(filePath,fullfile(path,'acado'));
       dirPath = fullfile(path,'acado',benchmark);
       if isdir(dirPath)
            rmdir(dirPath,'s');
       end
    end   

    % Gerneate path for files
    pathNew = fullfile(path,'acado',benchmark,Opts.extHorizon.decay);
    
    % Check if the folder exists and create it if not
    if ~exist(pathNew,'dir')
       mkdir(pathNew);
    end
    
    % Calculate the necessary number of files 
    if Opts.extHorizon.active
       if ischar(Opts.extHorizon.horizon) && strcmp(Opts.extHorizon.horizon,'all')
          horizon = Opts.Nc; 
       else
          horizon = Opts.extHorizon.horizon;
       end
    else
       horizon = 1; 
    end
    
    % Generate all files that do not exist yet
    for i = 1:horizon
       pathTempAcado = strcat(pathNew,filesep,'acado',sprintf('%i.m',i));
       if ~exist(pathTempAcado,'file') 
          WriteAcadoMainFiles(pathNew,text,Opts.nu,Opts.nx,i,Opts);
          compileAcadoFile(pathNew,i);
       end
    end
    
    % Save dynamics from the currently generated files
    if generate
        copyfile(fullfile(path,'acado',filename),fullfile(path,'acado',benchmark,filenameOld));
        delete(fullfile(path,'acado',filename));
    end
    
    % Add the path with the files to the Matlab-path
    addpath(pathNew);
end



%% Functions for writing files

function WriteAcadoMainFiles(path,text,nu,nx,index,Opts)

    %% Open file
    fileName = strcat('acado',sprintf('%i',index));
    filePath = strcat(path,filesep,fileName,'.m');
    fid = fopen(filePath,'w');

    %% ACADO Initilization

    fprintf(fid,'%s\n\n', '%% Begin ACADO Initialization');
    fprintf(fid,'%s\n\n','BEGIN_ACADO;');
    fprintf(fid,'%s\n','% Problem name');
    fprintf(fid,'%s\n\n\n', strcat('acadoSet(''problemname'', ''',fileName,''');'));

    %% Variables

    % States
    fprintf(fid,'%s\n\n', '%% Initialize variables');
    fprintf(fid,'%s\n','% States');

    counter = 1;
    for i = 1:index
       for j = 1:nx
           fprintf(fid,'%s\t%s\n','DifferentialState',strcat('x',sprintf('%i;',counter)));
           counter = counter +1;
       end
       fprintf(fid,'\n');
    end

    fprintf(fid,'%s\t%s\n\n','DifferentialState','inputcost;');

    % Parameters
    fprintf(fid,'%s\n','% Parameters (needed to couple differential states in constraints)');

    counter = 1;
    for i = 1:index-1
       for j = 1:nx
           fprintf(fid,'%s\t%s\n','Parameter',strcat('p',sprintf('%i;',counter)));
           counter = counter +1;
       end
       fprintf(fid,'\n');
    end

    fprintf(fid,'\n');

    % Control inputs
    fprintf(fid,'%s\n','% Control inputs');

    counter = 1;
    for i = 1:index
       for j = 1:nu
           fprintf(fid,'%s\t%s\n','Control',strcat('u',sprintf('%i;',counter)));
           counter = counter +1;
       end
       fprintf(fid,'\n');
    end
    
    fprintf(fid,'\n');


    %% Inputs to MEX-file

    fprintf(fid,'%s\n\n','%% Parse inputs to the MEX-file');

    inputCounter = 1;

    % x0

    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': x0'));
    inputCounter = inputCounter + 1;

    for i = 1:nx
       fprintf(fid,'%s\n',strcat('x0',sprintf('%i',i),' = acado.MexInput;'));
    end

    fprintf(fid,'\n%s','x0 = ');
    printVector(fid,nx,'x0',0);
    fprintf(fid,'\n\n'); 
    
    % xf
    
    for i = 1:index
        fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': xf(',sprintf('%i',i),')'));
        inputCounter = inputCounter + 1;
        
        nameTemp = sprintf('xf%i',i);
        
        for j = 1:nx
            fprintf(fid,'%s\n',strcat(nameTemp,'_',sprintf('%i',j),' = acado.MexInput;'));
        end

        fprintf(fid,'\n%s',strcat(nameTemp,' = '));
        printVector(fid,nx,strcat(nameTemp,'_'),0);
        fprintf(fid,'\n\n');       
    end
    
    % Matrix Q
    
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': Q'));
    inputCounter = inputCounter + 1;
    
    for i = 1:nx
        for j = 1:nx
            fprintf(fid,'%s\n',strcat('Q',sprintf('%i%i',i,j),' = acado.MexInput;'));
        end
        fprintf(fid,'\n');
    end
    
    fprintf(fid,'%s','Q = ');
    printMatrix(fid,nx,'Q');
    fprintf(fid,'\n\n'); 
    
    % Matrix R
    
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': R'));
    inputCounter = inputCounter + 1;
    
    for i = 1:nu
        for j = 1:nu
            fprintf(fid,'%s\n',strcat('R',sprintf('%i%i',i,j),' = acado.MexInput;'));
        end
        fprintf(fid,'\n');
    end
    
    fprintf(fid,'%s','R = ');
    printMatrix(fid,nu,'R');
    fprintf(fid,'\n\n'); 
    
    % multiple shooting steps
    
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': multiple shooting steps'));
    inputCounter = inputCounter + 1;
    
    fprintf(fid,'%s\n\n','Nc = acado.MexInput;');
    
    % optimization termination time T
    
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': optimization termination time T'));
    inputCounter = inputCounter + 1;
    
    fprintf(fid,'%s\n\n','t_end = acado.MexInput;');
    
    % u_max
    
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': u_max'));
    inputCounter = inputCounter + 1;

    for i = 1:nu
       fprintf(fid,'%s\n',strcat('u_max',sprintf('%i',i),' = acado.MexInput;'));
    end

    fprintf(fid,'\n%s','u_max = ');
    printVector(fid,nu,'u_max',0);
    fprintf(fid,'\n\n'); 
    
    % u_min
    
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': u_min'));

    for i = 1:nu
       fprintf(fid,'%s\n',strcat('u_min',sprintf('%i',i),' = acado.MexInput;'));
    end

    fprintf(fid,'\n%s','u_min = ');
    printVector(fid,nu,'u_min',0);
    fprintf(fid,'\n\n\n'); 

    
    %% Differential Equation

    fprintf(fid,'%s\n\n','%% Differential Equation');
    
    fprintf(fid,'%s\n','% Set the differential equation object for continous time in ACADO');
    fprintf(fid,'%s\n\n','f = acado.DifferentialEquation();');
    
    % System dynamics
    fprintf(fid,'%s\n','% System Dynamics');
    
    counterX = 1;
    counterU = 1;
    
    for i = 1:index
        tempText = text;
        % Replace States
        for j = 1:nx
            nameOld = sprintf('$x%i$',j);
            nameNew = sprintf('x%i',counterX);
            tempText = strrep(tempText,nameOld,nameNew);
            counterX = counterX + 1;
        end
        % Replace Controls
        for j = 1:nu
            nameOld = sprintf('$u%i$',j);
            nameNew = sprintf('u%i',counterU);
            tempText = strrep(tempText,nameOld,nameNew);
            counterU = counterU + 1;
        end
        
        fprintf(fid,tempText);
        fprintf(fid,'\n');
    end
    
    % Forming control vectors u
    fprintf(fid,'%s\n','% Forming control vectors u');
    
    counter = 0;
    for i = 1:index
        fprintf(fid,'\n%s',sprintf('u_%i = ',i));
        printVector(fid,nu,'u',counter);
        counter = counter + nu;
    end
    
    % Integration of control input costs 
    fprintf(fid,'\n\n%s\n','% Integration of control input costs');
    
    fprintf(fid,'%s','f.add(dot(inputcost) ==');
    fprintf(fid,'\t\t%s',sprintf('u_%i'' * R * u_%i',1,1));

    for i = 2:index
        if i == 2
           fprintf(fid,' ...\n'); 
        end
        if i ~= index
            fprintf(fid,'\t\t\t\t\t%s\n',sprintf('+ 1/%i * u_%i'' * R * u_%i ...',i,i,i));
        else
            fprintf(fid,'\t\t\t\t\t%s',sprintf('+ 1/%i * u_%i'' * R * u_%i',i,i,i));
        end
    end
    fprintf(fid,');\n\n\n');
    
    
    %% Optimal Control Problem
    
    fprintf(fid,'%s\n\n','%% Optimal Control Problem');
    
    fprintf(fid,'%s\n','% Parameters');
    fprintf(fid,'%s\t\t%s\n','start_time = 0;','% Set up start time of optimization');
    fprintf(fid,'%s\t%s\n','end_time = t_end;','% Set up termination time of optimization');
    fprintf(fid,'%s\t%s\n\n','grid_points = Nc;','% Set up gridpoints of optimization');
    
    fprintf(fid,'%s\n','% Set up the Optimal Control Problem (OCP) for ACADO');
    fprintf(fid,'%s\n\n','ocp = acado.OCP(start_time, end_time, grid_points);');
    
    % State vectors
    fprintf(fid,'%s\n','% Forming state vectors x');
    
    for i = 1:index
        fprintf(fid,'%s',strcat('x_',sprintf('%i',i),' = '));
        printVector(fid,nx,'x',(i-1)*nx);
        fprintf(fid,'\n');
    end
    
    % select weight function    
    decay = decayFunctions(index,Opts);
    
    % objective function
    fprintf(fid,'\n%s\n','% Define objective function');
    
    fprintf(fid,'%s\n',sprintf('objective = %i * (x_1(:,end)-xf1)''*Q*(x_1(:,end)-xf1);',decay(1)));

    for i = 2:index
        name1 = sprintf('x_%i',i);
        name2 = sprintf('xf%i',i);
        fprintf(fid,'%s\n',strcat('objective = objective + ',sprintf('%i * ',decay(i)),'(',name1,'(:,end)-',name2,')''*Q*(',name1,'(:,end)-',name2,');'));
    end
    
    % minimization term
    fprintf(fid,'\n%s\n','% Min(x,u) integral 0-T(inputcost) + Quadratic Distance Cost ');
    fprintf(fid,'%s\n\n\n','ocp.minimizeMayerTerm(inputcost + objective);');
    
    
    %% Constraints
    
    fprintf(fid,'%s\n\n','%% Constraints');
    
    % Differential equation
    fprintf(fid,'%s\n','% Optimize with respect to your differential equation');
    fprintf(fid,'%s\n\n','ocp.subjectTo( f );');
    
    % Initial point
    fprintf(fid,'%s\n','% Initial point');
    
    for i = 1:nx
        fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_START'', x',sprintf('%i',i),' ==  x0(',sprintf('%i',i),') );')); 
    end
    fprintf(fid,'\n');
    
    % Coupled boundary conditions
    counter1 = 0;
    counter2 = nx;
    
    for i = 1:index-1
        fprintf(fid,'%s\n',strcat('% Coupled boundary conditions',sprintf(' %i -> %i',i,i+1)));
        
        for j = 1:nx
            fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_END'', x',sprintf('%i - p%i',counter1+j,counter1+j),' ==  0 );')); 
        end
        
        fprintf(fid,'\n');
        
        for j = 1:nx
            fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_START'', x',sprintf('%i - p%i',counter2+j,counter1+j),' ==  0 );')); 
        end
        
        fprintf(fid,'\n');
        
        counter1 = counter1 + nx;
        counter2 = counter2 + nx;
    end
    
    % Initial value input-cost
    fprintf(fid,'%s\n','% Initial value for input costs L(0) = 0');
    fprintf(fid,'%s\n\n','ocp.subjectTo( ''AT_START'', inputcost ==  0.0 );');
    
    % Input constraints
    fprintf(fid,'%s\n','% Input constraints');
    
    counter = 1;
    
    for i = 1:index
       for j = 1:nu
           fprintf(fid,'%s\n',sprintf('ocp.subjectTo( u_min(%i) <= u%i <= u_max(%i) );',j,counter,j)); 
           counter = counter +1;
       end
       fprintf(fid,'\n');
    end
    
    
    %% Optimization Algorithm 
    
    fprintf(fid,'\n\n%s\n\n','%% Optimization Algorithm ');
    
    fprintf(fid,'%s\n','% Set up the optimization algorithm, link it to the OCP');
    fprintf(fid,'%s\n\n','algo =acado.OptimizationAlgorithm(ocp); ');
    
    fprintf(fid,'%s\n','% Set options for optimization');
    fprintf(fid,'%s\n','algo.set( ''HESSIAN_APPROXIMATION'', ''EXACT_HESSIAN'' );'); 
    fprintf(fid,'%s\n','algo.set( ''KKT_TOLERANCE'', 1e-11 );');
    fprintf(fid,'%s\n','algo.set( ''INTEGRATOR_TOLERANCE'', 1e-6 ); ');
    fprintf(fid,'%s\n','algo.set( ''INTEGRATOR_TYPE'', ''INT_RK45'' );');
    fprintf(fid,'%s\n','algo.set( ''DISCRETIZATION_TYPE'', ''MULTIPLE_SHOOTING'');');
    fprintf(fid,'%s\n\n','algo.set( ''ABSOLUTE_TOLERANCE'', 1e-6 );');
    
    fprintf(fid,'END_ACADO;');
    
    
    %% Close File

    fclose(fid);

end


%% Auxiliary functions

function printVector(fid,length,name,offset)
    fprintf(fid,'[');
    for i = 1:length
        if i ~= length
            fprintf(fid,'%s',strcat(name,sprintf('%i',i+offset),';'));
        else
            fprintf(fid,'%s',strcat(name,sprintf('%i',i+offset)));
        end
    end
    fprintf(fid,'];');
end

function printMatrix(fid,length,name)
    fprintf(fid,'%s','[');
    for i = 1:length
        for j = 1:length
            % print entries
            if j ~= length
                fprintf(fid,'%s',strcat(name,sprintf('%i%i',i,j),','));
            else
                fprintf(fid,'%s',strcat(name,sprintf('%i%i',i,j)));
            end
        end
        
        %printf line ending
        if i ~= length
            fprintf(fid,'%s\n\t','; ...');
        end
    end
    fprintf(fid,'];');
end

function compileAcadoFile(path,index)

    % change to the ACADO-Path
    pathTemp = cd;
    try 
        cd(path);
    end
    
    % delete parallel pool if one exists. This is necessary because it can
    % happen that the MEX-files from acado are stored in the parallel pool.
    % If this is the case, the ACADO files can not be created, because
    % access to the MEX-files is denied
    poolobj = gcp('nocreate');
    delete(poolobj);
   
    % compile the ACADO-files (call twice because otherwise the MEX-files
    % are not generated)
    command = sprintf('acado%i',index);
    evalin('base',command)
    evalin('base',command)
    
    % change back to the original path
    cd(pathTemp);
end