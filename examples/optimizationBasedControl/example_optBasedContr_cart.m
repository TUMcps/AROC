% This script demonstrates the control algorithm that is based on the
% optimization over reachable sets for the cart benchmark

% benchmark parameter
Param = param_cart(); 

% algorithm settings
Opts = [];

Opts.N = 10;                    % number of time steps
Opts.refTraj.Q = diag([5,5]);   % state weighting matrix (reference traj.)
Opts.refTraj.R = 1e-3;          % input weighthig matrix (reference traj.)

% controller synthesis
[objContr,res] = optimizationBasedControl('cart',Param,Opts);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
xlabel('x'); ylabel('v');