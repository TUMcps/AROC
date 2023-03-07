function R = reachPolyParam(R0,pZ_U,x0,u_c,Opts)
% REACHPOLYPARAM - computes the parameterized reachable set
% 
%
% Syntax:
%       R = REACHPOLYPARAM(R0,pZ_U,x0,u_c,Opts)
%
% Description:
%       Computes the approximate closed-loop reachable set parameterized 
%       by the control law paraemters.
%
% Input Arguments:
%
%       -R0:    initial set
%       -pZ_U:  polynomial zonotope that represents the polynomial control
%               law
%       -x0:    intial state for the reference trajectory
%       -u_c:   control inputs for the reference trajectory
%       -Opts:  structure containing all options
%
% Output Arguments:
%
%       -R:     reachable set (class: reachSet)
%
% See Also:
%       polynomialControl
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialization
    len = Opts.extHorizon.length;
    Ninter = Opts.Ninter;
    steps = Opts.reachSteps;
    options = Opts.cost.ReachOpts;
    params = Opts.cost.ReachParams;

    sets = repmat({pZ_U},len,1);

    % use center trajectory control input
    if Opts.refInput

        alpha_c = generators(Opts.U)\(u_c-center(Opts.U));

        ind = 1:Opts.np_s:Opts.Np;
        iMask = ismember(1:Opts.Np,ind);
        Idp0 = Opts.Idp(iMask,:);

        for i = 1:length(sets)
            alpha_i = alpha_c(:,(i-1)*Opts.Ninter+(1:Opts.Ninter));
            pZu_i = replaceId(sets{i},Opts.idp,Opts.Idp(:,i));
            sets{i} = resolve(pZu_i,alpha_i(:),Idp0(:,i));
        end
    end
    
    % interpolate center trajectory states and inputs
    fcent = @(x,u)Opts.funHandle(x,u,zeros(Opts.nw,1));
    [xc,uc] = interpolateCentTraj(u_c,steps,x0,fcent,Opts.dt);
    R = cell(len,1);

    % loop over all time steps
    for i = 1:len
        for j = 1:Ninter

            % extract set of control inputs
            dim_u = (j-1)*Opts.nu+(1:Opts.nu);
            U = project(sets{i},dim_u);

            % reachability parameter
            params.R0 = stack(R0,U);
            params.tStart = (j-1)*Opts.dt;
            params.tFinal = j*Opts.dt;
            params.refPoints = [xc(:,(j-1)*steps+1:j*steps+1);	
                                repmat(uc(:,(j-1)*steps+1),1,steps+1)];

            % compute reachable set
            Rtmp = reach(Opts.sys,params,options);

            % store results
            R0 = project(Rtmp.timePoint.set{end},1:Opts.nx);
            R{i} = R0;
        end
    end
end


% Auxiliary Functions -----------------------------------------------------

function [xc,uc] = interpolateCentTraj(Uc,Nr,x0,f,dt)
% interpolate reference trajectory

    uc = repelem(Uc,1,Nr);
    nx = length(x0);
    xc = zeros(nx,size(uc,2)+1);
    xc(:,1) = x0;
    
    for i=1:size(uc,2)
        [~,x_temp]=ode45(@(t,x)f(x,uc(:,i)),[0 dt/Nr],xc(:,i)');
        xc(:,i+1)=x_temp(end,:)';
    end
end