function im = animateCartpole(x,t,varargin)
% ANIMATECARTPOLE - animate the simulated trajectories
%
% Syntax:
%       ANIMATECARTPOLE(x,t)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       cartpole benchmark.
%
% Input Arguments:
%
%       -x:     states of the trajectory (dimension: [N,nx])
%       -t:     times for the trajectory (dimension: [N,1])
%
% See Also:
%       animate
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
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    im = [];

    % interpolate data
    N = ceil((t(end)-t(1))/0.05);
    t_ = linspace(t(1),t(end),N);
    
    [~,ind] = unique(t);

    x = interp1(t(ind),x(ind,:),t_);
    t = t_;

    % loop over all time steps
    for i = 1:size(x,1)

        hold on;

        % plot cartpole
        plotCartpole(x(i,:)');
        
        % formatting 
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([min(x(:,1))-1,max(x(:,1))+1]);
        ylim([-1.5,1.5]);

        if i < size(x,1)
            if nargout > 0
                I = getframe(gca); im{end+1} = I.cdata;
            end
            pause(t(i+1)-t(i));
            clf; hold off;
        end
    end
end

function plotCartpole(x)
% plot the cartpole at the current position

    % plot track
    patch([-200 -200 200 200],[-0.05 0.05 0.05 -0.05],'r', ...
                                          'FaceColor',[0.7 0.7 0.7]);

    % plot cart
    patch(x(1) + [-0.5 -0.5 0.5 0.5],[-0.15 0.15 0.15 -0.15],'b', ...
                                        'FaceColor',[0 102 255]/255);
    
    % plot pole
    phi = 0:0.01:2*pi;
    patch(x(1) + 0.1*cos(phi), 0.1*sin(phi),'k');
    
    R = [cos(x(2)) sin(x(2)); -sin(x(2)) cos(x(2))];
    v = R * [-0.02 -0.02 0.02 0.02; 0 1 1 0];
    patch(x(1) + v(1,:),v(2,:),'k');
end