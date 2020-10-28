% This script demonstrates the control algorithm that is based on the
% optimization over reachable sets for cart benchmark

% benchmark parameter
Param = param_cart(); 

% algorithm settings
Opts.N = 10;
Opts.refTraj.Q = diag([5,5]);
Opts.refTraj.R = 1e-3;

% controller synthesis
[objContr,res] = optimizationBasedControl('cart',Param,Opts);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plot(res.reachSetTimePoint{end},[1,2],'b');
plot(Param.R0,[1,2],'w','Filled',true);
xlabel('x');
ylabel('v');