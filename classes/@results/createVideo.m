function createVideo(obj,benchmark,file,varargin)
% CREATEVIDEO - create video from animation
%
% Syntax:
%       CREATEVIDEO(obj,benchmark,file)
%       CREATEVIDEO(obj,benchmark,file,statObs,dynObs,goalSet)
%       CREATEVIDEO(obj,benchmark,file,statObs,dynObs,goalSet,speedUp,addArg)
%
% Description:
%       Creates a video from the animation of the benchmark system and 
%       saves it to the specified file.
%
% Input Arguments:
%
%       -obj:           object of class results storing the simulated
%                       trajectories
%       -benchmark:     name of the considered benchmark model (see
%                       "aroc/benchmarks/...")
%       -file:          name of the file in which the video should be saved
%       -statObs:       cell-array storing the static obstacles
%       -dynObs:        cell-array storing the dynamic obstacles. Each
%                       dynamic obstacle is represented as a struct with 
%                       fields .set and .time 
%       -goalSet:       goal set for the motion planning problem
%       -speedUp:       speed-up factor for animation
%       -addArg:        additional arguments that are specific for the
%                       considered benchmark (e.g. lanelets for 'car'
%                       benchmark, see "aroc/benchmarks/animation)
%
% See Also:
%       results, animate
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

    % create video writer
    vid = VideoWriter(file, 'MPEG-4');
    vid.FrameRate = 1/0.05;
    open(vid);
    
    % animate the system
    im = animate(obj,benchmark,varargin{:});

    % write all frames to the video
    for i = 1:length(im)
        writeVideo(vid, im{i});
    end

    close(vid);
end