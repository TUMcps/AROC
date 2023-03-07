function [R, Rcont] = reachSafetyNetContr(sys,R0,H,U,Opts)
% REACHSAFETYNETCONTR - compute reachable set for the Safety Net Controller
%
% Syntax:
%       [R, Rcont] = REACHSAFETYNETCONTR(sys,R0,H,U,Opts)
%
% Description:
%       This function computes the forward reachble set for one time step
%       of the Safety Net Controller.
%
% Input Arguments:
%
%       -sys:   object that represents the dynamics of the closed-loop 
%               system (class: nonlinParamSys)
%       -R0:    initial set (class: zonotope)
%       -H:     matrix storing the generator-to-input assignment
%       -U:     cell-array containing the sets of admissble control inputs 
%               for all intermediate time steps
%       -Opts:  structure containing the following options
%
%           -.Ninter:       number of intermediate time steps
%           -.nu:           number of inputs
%           -.nx:           number of states
%           -.nGen:         number of generators
%           -.ReachSteps:   number of reachability steps
%           -.ReachOpts:    settings for reachability analysis with the
%                           CORA toolbox
%
% Output Arguments:
%
%       -R:         final reachable set (class: polyZonotope)
%       -Rcont:     object storing the reachable sets (class: reachSet)
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

    % initialization
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    Rcont = [];
    tStart = Opts.tStart;
    
    % add zero generators to the initial set so that system dim. is matched
    [c,G,E,H,indZero] = addZeroGen(R0,H,Opts);
    assignInput = eye(Opts.nGen);
    
    % loop over all intermediate time steps
    for i = 1:Opts.Ninter   
        
        % get center and generators of the input set
        cu = center(U{i});
        Gu = generators(U{i});
        
        % get current input-to-generator correlation matrix H
        ind = (Opts.Ninter-i)*Opts.nu;
        H_ = H(ind+1:ind+Opts.nu,:);
        
        % construct initial set for the extendet system dynamics
        c_ = [c; cu; zeros(Opts.nGen,1)];
        G_ = [G; Gu*H_*assignInput; assignInput];
        
        R0 = polyZonotope(c_,G_(:,1:size(E,2)),G_(:,size(E,2)+1:end),E);
        
        % consider measurement errors
        if ~isempty(Opts.V)
           R0 = R0 + cartProd(Opts.V,zeros(Opts.nu + Opts.nGen,1)); 
        end
        
        % update reachability parameter
        params.R0 = R0;
        params.tStart = tStart;
        params.tFinal = params.tStart + Opts.dt;
        tStart = params.tFinal;
        
        % compute the reachable set
        Rtemp = reach(sys,params,options);
        Rcont = add(Rcont,Rtemp);
        
        % extract the matrices from the final reachable set
        Rfin = Rtemp.timePoint.set{end};
        
        c = Rfin.c(1:Opts.nx);
        G_ = [Rfin.G,Rfin.Grest];
        G = G_(1:Opts.nx,:);
        assignInput = G_(Opts.nx+Opts.nu+1:end,:);
        E = Rfin.expMat;
        id = Rfin.id;
    end

    % remove factors that belong to all-zero generators from the set
    [c,G,Grest,E] = remZeroGen(c,G(:,1:size(E,2)),G(:,size(E,2)+1:end), ...
                               E,id,indZero);
    
    % construct the final reachable set
    R = polyZonotope(c,G,Grest,E);
    
end


% Auxiliary Functions -----------------------------------------------------

function [c,G,expMat,H,indZero] = addZeroGen(R0,H,Opts)
% This function adds zero generators to the initial set so that the number
% of generators matches the dimension of the nonlinSys object that
% represents the dynamics

    % extract properties of the initial set
    c = center(R0);
    G = generators(R0);
    expMat = eye(size(G,2));
    
    % add zero generators to the initial set so that system dim. is matched
    if size(G,2) < Opts.nGen
       indZero = size(G,2)+1:Opts.nGen;
       G = [G,zeros(Opts.nx,Opts.nGen-size(G,2))]; 
       expMat = [expMat,zeros(size(expMat,1),Opts.nGen-size(expMat,2))];
       H = [H,zeros(size(H,1),Opts.nGen-size(H,2))];
    else
       indZero = [];
    end
end

function [c,G,Grest,expMat] = remZeroGen(c,G,Grest,expMat,id,indZero)
% This function removes factors that belong to all-zero generators in the 
% initial set from the final reachable set
    
    if ~isempty(indZero)
        
        temp = ismember(id,indZero);
        indZero = find(temp == 1);
        ind = find(sum(expMat(indZero,:),1) ~= 0);
        
        if ~isempty(ind)
           
           % enclose the generators for initial all-zero gen. with zonotope
           pZ_ = polyZonotope(c,G(:,ind),Grest,expMat(:,ind));
           zono = zonotope(pZ_);
           Grest = generators(zono);
           c = center(zono);
           
           % keep the remaining generators
           indRest = setdiff(1:size(G,2),ind);
           expMat = expMat(:,indRest);
           G = G(:,indRest);
        end
    end
end