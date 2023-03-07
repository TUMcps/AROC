function writeAcadoFilesConfChecking(benchmark,Opts)
% WRITEACADOFILESCONFCHECKING - Generate files for ACADO Toolbox for
%                               conformant synthesis
%
% Syntax:
%       WRITEACADOFILESCONFCHECKING(benchmark,Opts)
%
% Description:
%       This function generates and compiles the files containing the
%       system dynamics, which are needed for solving optimal control
%       problems with the ACADO toolbox during conformant synthesis.
%
% Input Arguments:
%
%       -benchmark: name of the selected benchmark
%       -Opts:              a structure containing following options
%
%           -.group:   number of measurements grouped together
%           -.measErr: use measurement error to capture uncertainty
%           -.nx:      number of system states
%           -.nu:      number of system inputs
%
% See Also:
%       acadoConvertDynamics, writeAcadoFiles
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

    w = warning();
    warning('off','all');

    % file path where the files are generated
    [path,~,~] = fileparts(which('conformantSynthesis'));
    rmpath(genpath(fullfile(path,'acado')));

    % convert differential equations to ACADO format
    acadoConvertDynamics(path,benchmark,Opts)

    % read in the system dynamics
    if Opts.measErr
        id = ['Meas_',num2str(Opts.group)];  
    else
        id = ['No_',num2str(Opts.group)];
    end
    filename = 'acado.m';
    filePath = fullfile(path,'acado',benchmark,filename);
    text = fileread(filePath);
        
    % check if the system dynamics got changed
    generate = 0;
    filenameOld = ['acado_lastVersion_',id,'.m'];
    filePathLast = fullfile(path,'acado',benchmark,filenameOld);
    
    if ~exist(filePathLast,'file')
       generate = 1; 
    else
       textOld = fileread(filePathLast);
       textNew = fileread(filePath);
       if ~strcmp(textOld,textNew)
           generate = 1;
       else
           acFileName = fullfile(path,'acado',benchmark,['acadoConfChecking_',id,'.m']);
           if ~exist(acFileName,'file')
              generate = 1; 
           end 
       end
    end

    % generate path for files
    pathNew = fullfile(path,'acado',benchmark);
    
    % check if the folder exists and create it if not
    if ~exist(pathNew,'dir')
       mkdir(pathNew);
    end
    
    % generate ACADO files
    if generate 
      WriteAcadoMainFiles(pathNew,text,Opts.nu,Opts.nx,Opts.nw,Opts.measErr,Opts.group,id);
      compileAcadoFile(pathNew,Opts);
    end
    
    % save dynamics from the currently generated files
    if generate
        copyfile(fullfile(path,'acado',benchmark,filename),fullfile(path,'acado',benchmark,filenameOld));
        delete(fullfile(path,'acado',benchmark,filename));
    end
    
    % add the path with the files to the Matlab-path
    addpath(pathNew);
    
    warning(w);
end


% Auxiliary Functions -----------------------------------------------------

function acadoConvertDynamics(path,benchmark,Opts)
% convert the dynamics to ACADO foramt

    % function handle to dynamic file
    str = ['funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % open file to which the dynamic function in ACADO format is written
    benchPath = fullfile(path,'acado',benchmark);
    if ~exist(benchPath,'dir')
       mkdir(benchPath);
    end
    
    filename = 'acado.m';
    filePath = fullfile(benchPath,filename);
    fid = fopen(filePath,'w');
    
    % express the system dynamics as a symbolic function
    x = sym('X',[Opts.nx,1]);
    u = sym('U',[Opts.nw,1]);
    w = sym('W',[Opts.nw,1]);
    
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
       
       % replace disturbance variables
       for j = 1:Opts.nw
          oldStr = sprintf('W%i',j);
          newStr = sprintf('$w%i$',j);
          str = strrep(str,oldStr,newStr);
       end
       
       % convert to acado format
       strTemp = sprintf('dot($x%i$)',i);
       str = ['f.add(',strTemp,' == 1/$t$*(',str,'));'];
       
       % write to file
       fprintf(fid,'%s\n', str);
       
    end
    
    % close file
    fclose(fid);
end

function WriteAcadoMainFiles(path,text,nu,nx,nw,measErr,index,id)

    %% Open file
    fileName = ['acadoConfChecking_',id];
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
           fprintf(fid,'%s %s\n','DifferentialState',strcat('x',sprintf('%i;',counter)));
           counter = counter +1;
       end
       fprintf(fid,'\n');
    end


    % Control inputs (= Disturbances for conformance checking)
    fprintf(fid,'%s\n','% Control inputs (= Disturbances)');
    
    counter = 1;
    for i = 1:index
        for j = 1:nw
           fprintf(fid,'%s\t%s\n','Control',strcat('w',sprintf('%i;',counter)));
           counter = counter + 1;
        end
        fprintf(fid,'\n');
    end
    
    if measErr
        counter = 1;
        for i = 1:index
            for j = 1:nx
                fprintf(fid,'%s\t%s\n','Control',strcat('v',sprintf('%i;',counter)));
                counter = counter + 1;
            end
            fprintf(fid,'\n');
        end
    end
    
    % Parameters
    fprintf(fid,'%s\n','% Parameters (needed to couple differential states in constraints)');

    counter = 1;
    for i = 1:index-1
       for j = 1:nx
           fprintf(fid,'%s %s\n','Parameter',strcat('p',sprintf('%i;',counter)));
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
    
    % xf
    
    fprintf(fid,'\n%s\n',strcat('% Input',sprintf(' %i',inputCounter),': xf'));
    inputCounter = inputCounter + 1;

    counter = 1;
    for i = 1:index
       for j = 1:nx
           fprintf(fid,'%s\n',strcat('xf',sprintf('%i',counter),' = acado.MexInput;'));
           counter = counter +1;
       end
       fprintf(fid,'\n');
    end
    
    % u
    
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': u'));
    inputCounter = inputCounter + 1;

    counter = 1;
    for i = 1:index
       for j = 1:nu
           fprintf(fid,'%s\n',strcat('u',sprintf('%i',counter),' = acado.MexInput;'));
           counter = counter +1;
       end
       fprintf(fid,'\n');
    end

    % dt

    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': dt'));
    inputCounter = inputCounter + 1;

    for i = 1:index
       fprintf(fid,'%s\n',strcat('dt',sprintf('%i',i),' = acado.MexInput;'));
    end

    % optimization termination time T

    fprintf(fid,'\n%s\n',strcat('% Input',sprintf(' %i',inputCounter),': optimization termination time T'));
    inputCounter = inputCounter + 1;
    fprintf(fid,'%s\n\n','t_end = acado.MexInput;');

    % trade-off parameter
    fprintf(fid,'%s\n',strcat('% Input',sprintf(' %i',inputCounter),': trade-off parameter mu'));
    fprintf(fid,'%s\n\n','mu = acado.MexInput;');


    
    %% Differential Equation

    fprintf(fid,'\n%s\n\n','%% Differential Equation');
    
    fprintf(fid,'%s\n','% Set the differential equation object for continous time in ACADO');
    fprintf(fid,'%s\n\n','f = acado.DifferentialEquation();');
    
    % System dynamics
    fprintf(fid,'%s\n','% System Dynamics');
    
    counterX = 1; counterU = 1; counterW = 1;
    
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
        % Replace Disturbances
        for j = 1:nw
            nameOld = sprintf('$w%i$',j);
            nameNew = sprintf('w%i',counterW);
            tempText = strrep(tempText,nameOld,nameNew);
            counterW = counterW + 1;
        end
        % Replace Time Step
        tempText = strrep(tempText,'$t$',sprintf('dt%i',i));
        
        fprintf(fid,tempText);
        fprintf(fid,'\n');
    end
    
    %% Optimal Control Problem
    
    fprintf(fid,'\n%s\n\n','%% Optimal Control Problem');
    
    fprintf(fid,'%s\n','% Parameters');
    fprintf(fid,'%s\t\t%s\n','start_time = 0;','% Set up start time of optimization');
    fprintf(fid,'%s\t%s\n','end_time = t_end;','% Set up termination time of optimization');
    fprintf(fid,'%s\t%s\n\n','grid_points = 1;','% Set up gridpoints of optimization');
    
    fprintf(fid,'%s\n','% Set up the Optimal Control Problem (OCP) for ACADO');
    fprintf(fid,'%s\n','ocp = acado.OCP(start_time, end_time, grid_points);');
    
    % objective function
    fprintf(fid,'\n%s\n','% Define objective function');
    
    text = 'objective = mu*w1*w1 ';
    
    for i = 2:index*nw
        text = [text, sprintf('+ mu*w%i*w%i ',i,i)];
    end
    
    if measErr
        for i = 1:index*nx
            text = [text, sprintf('+ (1-mu)*v%i*v%i ',i,i)];
        end 
    end
    
    fprintf(fid,'%s;\n',text);
    
    % minimization term
    fprintf(fid,'\n%s\n','% Min(x,u) w''*w ');
    fprintf(fid,'%s\n\n\n','ocp.minimizeMayerTerm(objective);');
    
    
    %% Constraints
    
    fprintf(fid,'%s\n\n','%% Constraints');
    
    % Differential equation
    fprintf(fid,'%s\n','% Optimize with respect to your differential equation');
    fprintf(fid,'%s\n\n','ocp.subjectTo( f );');
    
    % Initial point
    fprintf(fid,'%s\n','% Initial point');
    
    for i = 1:nx
        fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_START'', x',sprintf('%i',i),' == x0',sprintf('%i',i),');')); 
    end
    fprintf(fid,'\n');
    
    % Coupled boundary conditions
    counter1 = 0;
    counter2 = nx;
    
    for i = 1:index-1
        fprintf(fid,'%s\n',strcat('% Coupled boundary conditions',sprintf(' %i -> %i',i,i+1)));
        
        for j = 1:nx
            fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_END'', x',sprintf('%i - p%i',counter1+j,counter1+j),' == 0 );')); 
        end
        
        fprintf(fid,'\n');
        
        for j = 1:nx
            fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_START'', x',sprintf('%i - p%i',counter2+j,counter1+j),' == 0 );')); 
        end
        
        fprintf(fid,'\n');
        
        counter1 = counter1 + nx;
        counter2 = counter2 + nx;
    end
    
    % Final point
    fprintf(fid,'%s\n','% Final point');
    
    counter = 1;
    
    for i = 1:index
        for j = 1:nx
            if measErr
                fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_END'', x',sprintf('%i',counter),' + v',sprintf('%i',counter),' == xf',sprintf('%i',counter),');')); 
            else
                fprintf(fid,'%s\n',strcat('ocp.subjectTo( ''AT_END'', x',sprintf('%i',counter),' == xf',sprintf('%i',counter),');')); 
            end
            counter = counter + 1;
        end
        fprintf(fid,'\n');        
    end
    
    
    %% Optimization Algorithm 
    
    fprintf(fid,'\n%s\n\n','%% Optimization Algorithm ');
    
    fprintf(fid,'%s\n','% Set up the optimization algorithm, link it to the OCP');
    fprintf(fid,'%s\n\n','algo = acado.OptimizationAlgorithm(ocp); ');
    
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

function compileAcadoFile(path,Opts)

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
    if Opts.measErr
        command = ['acadoConfChecking_Meas_',num2str(Opts.group)];
    else
        command = ['acadoConfChecking_No_',num2str(Opts.group)];
    end
    evalin('base',command)
    evalin('base',command)
    
    % change back to the original path
    cd(pathTemp);
end