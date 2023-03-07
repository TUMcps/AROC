function [W,V] = conformantSynthesisLinear(A,B,D,c,measurements,Opts)
% CONFORMANTSYNTHESISLINEAR - conformant synthesis for linear models
%
% Syntax:
%       [W,V] = CONFORMANTSYNTHESISLINEAR(A,B,D,c,measurements,Opts)
%
% Description:
%       This function computes the model uncertainty that is required such 
%       that the uncertain model encloses all measurements of the real 
%       system. The model uncertainty consists of a set of uncertain inputs
%       W and a set of measurement errors V.
%
% Input Arguments:
%
%       -A:     system matrix of the linear model (dimension: [nx,nx])
%       -B:     input matrix of the linear model (dimension: [nx,nu])
%       -D:     disturbance matrix of the linear model (dimension: [nu,nw])
%       -c:     constand offset of the linear model (dimension: [nx,1])
%       -measurements: cell-array storing the measurements of the real
%                      system, where each cell is a struct with the 
%                      following fields: 
%
%           -.x:       matrix storing the measured trajectory of the real
%                      system (dimension: [n,N])
%           -.u:       matrix storing the input signal corresponding to the
%                      measured trajectory (dimension: [m,N-1])
%           -.t:       vector storing the time points for the measured
%                      trajectory (dimension: [1,N])
%
%       -Opts:         a structure containing the algorithm settings
%
%           -.set:     string specifying the type of set which is used to
%                      represent the uncertainty.
%                      [{'interval'} / 'zonotope']
%           -.group:   number of measurements grouped together
%                      [{1} / positive integer]
%           -.measErr: use measurement error to capture uncertainty
%                      [{false} / true]
%           -.mu       tradeoff between uncertain inputs and measurement
%                      errors
%                      [{0.5} / double between 0 and 1]
%
% Output Arguments:
%
%       -W:     resulting set of uncertain inputs
%       -V:     resulting set of measurement errors
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

    nx = size(A,1); nw = size(D,2);

    % check if the data is uniformly sampled
    uniform = true;

    for i = 1:length(measurements)
        tmp = diff(measurements{i}.t);
        if abs(mean(tmp) - max(tmp)) < 1e-15
            uniform = false;
        end
    end

    % tranform to a discrete time system
    if uniform
        dt = mean(diff(measurements{1}.t));
        [A_,B_,D_,c_] = discreteTimeSystem(A,B,D,c,dt);
    end
    
    % initialization
    options = optimoptions('quadprog','display','off');
    W = zeros(nw,1); V = zeros(nx,1);

    % loop over all measurements
    for i = 1:length(measurements)
       
        % split data into smaller chunks
        chunks = 1:Opts.group:size(measurements{i}.x,2);
        if chunks(end) ~= size(measurements{i}.x,2)
           chunks = [chunks size(measurements{i}.x,2)];
        end
        
        x0 = measurements{i}.x(:,1);
        
        % loop over all data chunks
        for j = 1:length(chunks)-1
           
            X_ = measurements{i}.x(:,chunks(j)+1:chunks(j+1));
            U_ = measurements{i}.u(:,chunks(j):chunks(j+1)-1);
            T_ = diff(measurements{i}.t(chunks(j):chunks(j+1)));
            
            % check if measurement is already contained in reachable set
            %     ==> no need to update uncertainty
            if (i ~= 1 || j ~= 1) && Opts.measErr && uniform
                
                points = zeros(nx,Opts.group);
                
                if any(rad(W) > 0)
                
                    Wzono = zonotope(W); 

                    for k = 1:size(X_,2)
                        R = A_*R + B_*U_(:,k) + c_ + D_*Wzono;
                        p = getClosestPoint(R,X_(:,k));
                        points(:,k) = X_(:,k) - p;
                    end

                    R = reduceUnderApprox(R,'sum',5);
                    
                else     
                    for k = 1:size(X_,2)
                        R = A_*R + B_*U_(:,k) + c_ + D_*center(W);
                        points(:,k) = X_(:,k) - R;
                    end
                end
                
                if contains(V,points)
                    if isa(R,'zonotope')
                        x0 = center(R);
                    else
                        x0 = R;
                    end
                    continue;
                end
            end
            
            % construct equality constraints H*x <= d for linear program
            H1 = []; H2 = []; d = []; x = x0;
            
            for k = 1:size(X_,2)
                if ~uniform
                    dt = T_(k);
                    [A_,B_,D_,c_] = discreteTimeSystem(A,B,D,c,dt);
                    if k == 1
                        Dall = D_;
                    else
                        Dall = [A_*Dall, D_];
                    end
                end
                H1 = [H1 zeros(size(H1,1),nw);Dall];
                H2 = blkdiag(H2,eye(nx));
                x = A_*x + B_*U_(:,k) + c_;
                d = [d; X_(:,k) - x];
            end
            
            if Opts.measErr
                H = [H1 H2];
            else
                H = [H1 0*H2];
            end
            x0 = x;
            
            % construct objective function min_x f*x for linear program
            f = [Opts.mu*ones(size(H1,2),1); ...
                                    (1-Opts.mu)*ones(size(H2,2),1)];
            
            % solve linear program
            x = quadprog(diag(f),[],[],[],H,d,[],[],[],options);

            if isempty(x)
                error('Conformance checking failed!');
            end
            
            % extract values for disturbances w_i and measurement err. v_i
            temp = blkdiag(eye(size(H1,2)), eye(size(H2,2)))*x;
            w = temp(1:size(H1,2)); v = temp(size(H1,2)+1:end);
            w = reshape(w,[nw,length(w)/nw]);
            v = reshape(v,[nx,length(v)/nx]);
            
            % update sets for disturbances and measurement errors
            if strcmp(Opts.set,'interval')
                W = W | interval(min(w,[],2),max(w,[],2));
                V = V | interval(min(v,[],2),max(v,[],2));
            else
                W = [W, w]; V = [V, v];
            end
            
            % initialize reachable set in the first iteration
            if i == 1 && j == 1
               R = x0;
            end
        end
    end

    % enclose point cloud with a zonotope
    if ~strcmp(Opts.set,'interval')
       W = zonotope.enclosePoints(W,'maiga'); 
       if Opts.measErr
          V = zonotope.enclosePoints(V,'maiga'); 
       end
    end

    % assign output arguments 
    if ~Opts.measErr
        V = [];
    end
end


% Auxiliary Functions -----------------------------------------------------

function p = getClosestPoint(R,p)
% estimate for the point closest to p in the zonotope R

    c = p - R.Z(:,1);
    alpha = sign(c'*R.Z(:,2:end));
    p = R.Z(:,1) + R.Z(:,2:end)*alpha';
end

function [A,B,D,c] = discreteTimeSystem(A,B,D,c,dt)
% convert a continuous time linear system to a discrete time one

    m = size(B,2);
    sys = linearSysDT(linearSys(A,[B,D],[],c),dt);
    A = sys.A; B = sys.B(:,1:m); D = sys.B(:,m+1:end); c = sys.c;
end