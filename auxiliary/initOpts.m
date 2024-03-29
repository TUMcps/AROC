function Opts = initOpts(name,benchmark,Opts,Param)
% INITOPTS - initialize algorithm settings
%
% Syntax:
%       Opts = INITOPTS(name,benchmark,Opts,Param)
%
% Description:
%       Initializes algorithm settings and parameters. In particular, this
%       function checks if all user provided settings take valid values, it
%       generates the files for the ACADO toolbox, and it initializes the
%       settings for computing references trajectories.
%
% Input Arguments:
%
%       -name:      name of the control algorithm, e.g.
%                   'generatorSpaceControl', etc.
%       -benchmark: name of the considered benchmark model (see
%                   "aroc/benchmarks/...")
%       -Opts:      a structure containing the algorithm settings
%       -Param:     a structure containing the benchmark parameters
%
% Output Arguments:
%
%       -Opts:      adapted structure of algorithm settings
%
% See Also:
%       generatorSpaceControl, convexInterpolationControl
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
    str = ['funHan = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % get number of states, inputs, and disturbances
    if isfield(Opts,'nx') && isfield(Opts,'nu') && isfield(Opts,'nw')
        nx = Opts.nx; nu = Opts.nu; nw = Opts.nw;
        Opts = rmfield(Opts,'nx');
        Opts = rmfield(Opts,'nu');
        Opts = rmfield(Opts,'nw');
    else
        [count,out] = inputArgsLength(funHan,3);

        nx = out;
        nu = count(2);
        nw = count(3);
    end
    
    % check if the model is linear or nonlinear
    isLin = 0;
    
    if any(ismember({'optimizationBasedControl','combinedControl'},name))
        [isLin,A,B,D,c] = isLinearModel(funHan,nx,nu,nw);
    end
    
    % check if the specified parameters are valid
    Param = checkParam(Param,name,nx,nu,nw);
    Opts = checkOpts(Opts,name,Param,nx,nu,isLin);
    
    % copy benchmark parameter to options
    Opts = params2options(Param,Opts);
    Opts.funHandle = funHan;
    Opts.nx = nx;
    Opts.nu = nu;
    Opts.nw = nw;
    Opts.U = zonotope(Opts.U);
    if isfield(Opts,'V')
       Opts.V = zonotope(Opts.V);
    end
    
    if isfield(Opts.cora,'alg') && strcmp(Opts.cora.alg,'poly')
        Opts.R0 = polyZonotope(Opts.R0);
    else
        Opts.R0 = zonotope(Opts.R0);
    end  
    
    if isLin
        Opts.linModel.A = A;
        Opts.linModel.B = B;
        Opts.linModel.D = D;
        Opts.linModel.c = c;
    end
    
    % options for reachability analysis with the CORA toolbox
    Opts.ReachOpts = Opts.cora;
    Opts = rmfield(Opts,'cora');
    Opts.ReachParams.U = zonotope(Opts.W);

    % settings for optimal control to compute the reference trajectory
    fields = {'nx','nu','nw','tFinal','xf'};
    CentOpts = [];
    for fn = fields
       CentOpts.(fn{1}) = Opts.(fn{1}); 
    end
    
    if isfield(Opts,'refTraj')             
        if isfield(Opts.refTraj,'Q') && isfield(Opts.refTraj,'R')
            CentOpts.Q = Opts.refTraj.Q;
            CentOpts.R = Opts.refTraj.R;
        else
            CentOpts.refTraj = Opts.refTraj; 
        end
    end
    
    CentOpts.x0 = center(Opts.R0); 
    CentOpts.uMax = supremum(Param.U);
    CentOpts.uMin = infimum(Param.U);
    
    % generate ACADO or Fmincon files
    [path,~,~] = fileparts(which(name));
    
    if isempty(which('BEGIN_ACADO'))
        
        CentOpts.useAcado = 0;
        
        if isfield(Opts,'extHorizon')
            CentOpts.extHorizon = Opts.extHorizon;
            CentOpts.extHorizon.horizon = 1;
        end
        
        if strcmp(name,'generatorSpaceControl') || ...
           strcmp(name,'safetyNetControl') || strcmp(name,'polynomialControl')
            CentOpts.Ninter = Opts.N * Opts.Ninter;
        else
            CentOpts.Ninter = Opts.N; 
        end
        
        w = warning();
        warning('off','all');

        rmpath(genpath(fullfile(path,'fmincon')));
        
        writeFminconFiles(path,benchmark,CentOpts);
        
        if strcmp(name,'convexInterpolationControl')
            CentOpts.Ninter = Opts.Ninter;
            CentOpts.extHorizon = Opts.extHorizon;
            writeFminconFiles(path,benchmark,CentOpts);
        end

        if strcmp(name,'safetyNetControl')
            if iscell(Opts.controller)
                for i = 1:length(Opts.controller)
                    if strcmp(Opts.controller{i},'MPC')
                        for j = 1:Opts.contrOpts{i}.horizon
                            CentOpts.Ninter = j * Opts.contrOpts{i}.Ninter;
                            writeFminconFiles(path,benchmark,CentOpts);
                        end
                    end
                end
            elseif strcmp(Opts.controller,'MPC')
                for j = 1:Opts.contrOpts.horizon
                    CentOpts.Ninter = j * Opts.contrOpts.Ninter;
                    writeFminconFiles(path,benchmark,CentOpts);
                end
            end
        end

        if strcmp(name,'polynomialControl') && Opts.refUpdate
            for j = 1:Opts.N
                CentOpts.Ninter = j*Opts.Ninter;
                writeFminconFiles(path,benchmark,CentOpts);
            end
        end
        
        CentOpts = rmfield(CentOpts,'Ninter');
        
        warning(w);
        
    else
        
        CentOpts.useAcado = 1;
        
        if isfield(Opts,'extHorizon') && ...
                strcmp(name,'convexInterpolationControl')
            CentOpts.extHorizon = Opts.extHorizon;
            CentOpts.Nc = Opts.N;
        end

        w = warning();
        warning('off','all');

        rmpath(genpath(fullfile(path,'acado')));

        acadoConvertDynamics(path,CentOpts,benchmark);
        writeAcadoFiles(path,benchmark,CentOpts);

        warning(w);
    end
    
    Opts.CentOpts = CentOpts;
end