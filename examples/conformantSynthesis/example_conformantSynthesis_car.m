% This script demonstrates conformance checking using measurements from an
% autonomous car and compared different set representations for the model
% uncertainties

% load measurements from the car
load('measurements_car');

% algorithm settings
Opts = [];
Opts.group = 6;         % number of measurement grouped together
Opts.measErr = true;    % use measurement errors to represent uncertainty

% conformance checking (interval)
clock = tic;
Opts.set = 'interval';
[Wint,Vint] = conformantSynthesis('car',M,Opts);
tComp = toc(clock);

disp(['Computation time (interval): ',num2str(tComp)]);

% conformance checking (zonotope)
clock = tic;
Opts.set = 'zonotope';
[Wzono,Vzono] = conformantSynthesis('car',M,Opts);
tComp = toc(clock);

disp(['Computation time (zonotope): ',num2str(tComp)]);

% visualization
figure; hold on; box on;
h1 = plot(Wint,[1,2],'b');
h2 = plot(Wzono,[1,2],'r');
legend([h1,h2],'interval','zonotope');
xlabel('w_1'); ylabel('w_2');

figure; hold on; box on;
R = reachSetConformance('car',M{1},Wint,Vint);
plot(R,[3,4],'FaceColor',[.6 .6 .6],'EdgeColor','none');
plot(M{1}.x(3,:),M{1}.x(4,:),'k');
xlabel('x'); ylabel('y');