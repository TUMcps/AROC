function [statObs,dynObs,x0,goalSet,waterways,shallows,information] = commonocean2cora(filename,varargin)
% COMMONOCEAN2CORA - import CommonOcean xml-file into AROC
%
% Syntax:
%       [statObs,dynObs,x0,goalSet,waterways, shallows, information] = commonocean2cora(filename)
%
% Description:
%       This function reads a motion planning problem for an autonomous 
%       ship consisting of static and dynamic obstacles as well as an 
%       initial state and a goal set from a CommonOcean xml-file.
%
% Input Arguments:
%
%       -filename:       name of the CommonOcean XML-file scenario
%
% Output Arguments:
%
%       -statObs:        cell-array storing the static obstacles
%       -dynObs:         cell-array storing the set and the time interval 
%                        for the dynamic obstacles
%       -x0:             initial point for the planning problem
%       -goalSet:        cell-array storing the goal sets and the 
%                        corresponding time
%       -waterways:      cell-array storing the sets for all waterways
%       -shallows:       cell-array storing the shallows
%       -information:    contains further information extracted from 
%                        CommonOcean
%
% See Also:
%       maneuverAutomaton
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
% Authors:      Farah Atour, Niklas Kochdumper, Philipp Gassert, Leni Rohe 
%               Hanna Krasowski, Bruno Maione
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2022 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 
    
    conTimerStart = tic;
    
    % ----------------------------
    % ----- Reading options ------
    % ----------------------------
    
    defaultVerbose = true;
    
    p = inputParser;
    addParameter(p, 'verbose', defaultVerbose, @(x) islogical(x) || (x == 1) || (x == 0) );
    
    parse(p, varargin{:});
    
    verbose = p.Results.verbose;
    
    % ----------------------------
    % --- Reading the XML file ---
    % ----------------------------
    
    % add '.xml' to the filename if missing
    if ~contains(filename, '.xml')
        filename = strcat(filename,'.xml');
    end
    xDoc = xmlread(filename);
    
    % assert number of scenarios
    assert(xDoc.getElementsByTagName('commonOcean').getLength == 1, 'Cannot handle more than one scenario!');
    
    % --- Retreive commonOcean version ---
    commonocean_version = xDoc.getElementsByTagName('commonOcean').item(0).getAttribute('commonOceanVersion');
    
    % -----------------------------------------------
    % -- Extract all static and dynamic obstacles ---
    % -----------------------------------------------
    
    timeStep = str2double(xDoc.item(0).getAttribute('timeStepSize'));
    dynamic_counter = 0;         % counts number of dynamic obstacles
    static_counter  = 0;         % counts number of static obstacles
    shallows_counter = 0;         % counts number of shallows
    total_dynamic_counter = 0;   % counts total number of all (time independet) dynObs
    statObs = [];
    dynObs = [];
    shallows = [];
    boundaryMessage_flag = 1;
    
    w = warning();
    warning('off');
    
    % --- Extracting obstacles for 2022a version ---
    if strcmp(commonocean_version, '2022a')
        dynamicObstaclesList = xDoc.getElementsByTagName('dynamicObstacle');
        dynamicObstacles_length = dynamicObstaclesList.getLength;
        staticObstaclesList = xDoc.getElementsByTagName('staticObstacle');
        staticObstacles_length = staticObstaclesList.getLength;
        dynObsMat = [];
       
        counterBadOnes = 0;
        for i = 0:(dynamicObstacles_length-1)
            dynamic_counter = dynamic_counter + 1;
            
            % extracting dynamic obstacles
            dynamicBuffer = dynamic_fct(dynamicObstaclesList, i);
            total_dynamic_counter = total_dynamic_counter + length(dynamicBuffer);
            
            for j = 1:length(dynamicBuffer)
                % get set
                points = dynamicBuffer(j).polygon;
                dynObs{end+1,1}.set = polygon(points(:,1),points(:,2));
                
                % get time interval;
                t = dynamicBuffer(j).time*timeStep;
                if length(t) == 1
                    dynObs{end,1}.time = interval(t);
                else
                    dynObs{end,1}.time = interval(t(1),t(2));
                end
            end

            % extracting dynamic obstracles matrix
            obsmat = dynamic_mat(dynamicObstaclesList , timeStep, i);

            % Filter out the ones we dont want based on ship length and max
            % velocity difference
            vel = sqrt(obsmat(:,6).^2 + obsmat(:,7).^2);
            acc = [];
            for j = 1:length(vel)-1
                acc(j) = (vel(j+1) - vel(j)) / timeStep; % acc
            end
            if max(acc) > 10 || min(acc) < -10
                counterBadOnes = counterBadOnes + 1;
                continue
            end
            % shape = getXList(dynamicObstaclesList, {'shape'}, i);
            % ship_length = getXEl(shape, {'rectangle', 'length'});
            % if ship_length < 250
            %     counterBadOnes = counterBadOnes + 1;
            %     continue
            % end
            dynObsMat = [dynObsMat; obsmat];            
        end 

        % extracting static obstacles
        for i = 0:(staticObstacles_length-1)
            
            % initial state
            ShapeList_initialstate = getXList(staticObstaclesList, {'shape'}, i);
            stateList_initialstate = getXList(staticObstaclesList, {'initialState'}, i);
            
            % transforming any shape into a polyshape
            initialstate_vertices = shape_fct(ShapeList_initialstate, stateList_initialstate);
            initialstate_polyshape = polyshape(initialstate_vertices); 
            
            % split into regions to obtain single-regions obstacles
            reg = regions(initialstate_polyshape);
            
            if length(reg) > 1
                fprintf('More than one region in static obstacle.\n')
            end
            
            for l = 1:size(reg,1)
                static_counter = static_counter + 1;
                vert = reg(l,1).Vertices;
                statObs{end+1,1} = polygon(vert(:,1),vert(:,2));
            end
        end
    else
        fprintf('Unidentified version (not 2022a) when extracting obstacle information. No obstacles extracted.');
    end
    
    % -----------------------------------------------
    % -- Extract all waterways of the Waters Network ---
    % -----------------------------------------------
    
    watersList = xDoc.getElementsByTagName('waterway');
    
    % --- Preallocate for better memory and speed ---
    waters_length = watersList.getLength();
    generalWatersList = struct('id', cell(waters_length,1), ...
        'leftBound', cell(waters_length,1), ...
        'rightBound', cell(waters_length,1));
    waterways = cell(waters_length,logical(waters_length));
    
    for i = 0:(waters_length-1)
        
        % retrieve id and set to -1 for references from goal region specification
        if ~strcmp(watersList.item(i).getAttribute('id'),'')
            generalWatersList(i+1).id = str2double(watersList.item(i).getAttribute('id'));
        else
            generalWatersList(i+1).id = -1;
            continue
        end
        
        % left bound
        leftbound_pointList = getXList(watersList, {'leftBound', 'point'}, i);
        generalWatersList(i+1).leftBound = zeros(2,leftbound_pointList.getLength());
        
        for j = 0:(leftbound_pointList.getLength()-1)
            generalWatersList(i+1).leftBound(1,j+1) = getXEl(leftbound_pointList, {'x'}, j);
            generalWatersList(i+1).leftBound(2,j+1) = getXEl(leftbound_pointList, {'y'}, j);
        end
        
        % right bound
        rightbound_pointList = getXList(watersList, {'rightBound', 'point'}, i);
        generalWatersList(i+1).rightBound = zeros(2,rightbound_pointList.getLength());
        
        for j = 0:(rightbound_pointList.getLength()-1)
            generalWatersList(i+1).rightBound(1,j+1) = getXEl(rightbound_pointList, {'x'}, j);
            generalWatersList(i+1).rightBound(2,j+1) = getXEl(rightbound_pointList, {'y'}, j);
        end
        
        % vertices of the left bound lanelet (eases handling below)
        x_left = generalWatersList(i+1).leftBound(1,:);
        y_left = generalWatersList(i+1).leftBound(2,:);
        
        % vertices of the right bound lanelet (eases handling below)
        x_right = generalWatersList(i+1).rightBound(1,:);
        y_right = generalWatersList(i+1).rightBound(2,:);
        
        % insertion into output cell array
        waterways{i+1} = polygon([x_left, flip(x_right)], [y_left, flip(y_right)]);
        if waterways{i+1}.set.NumRegions > 1
            fprintf(strcat('Warning: Waterways consists of %d regions!',...
                ' Assuming imprecision contained in pointlist input file.',...
                ' Continuing without alteration.\n'), waterways{i+1}.set.NumRegions);
        end
    end
       
    % delete non-waters-constituting waters (i.e. waters references in goal
    % regions that were picked up by getElementsByTagName)
    for i = waters_length:-1:1
        if generalWatersList(i).id == -1
            generalWatersList(i) = [];
            waterways(i) = [];
        end
    end
    
    % -----------------------------------------------
    % -- Extract all shallows of the Waters Network ---
    % -----------------------------------------------
    
    shallowsList = xDoc.getElementsByTagName('shallow');
    shallows_length = shallowsList.getLength();
    
    for i = 0:(shallows_length-1)

        % initial state
        ShapeList_initialstate = getXList(shallowsList, {'shape'}, i);
        
        % transforming any shape into a polyshape
        initialstate_vertices = shape_fct(ShapeList_initialstate);
        initialstate_polyshape = polyshape(initialstate_vertices); 

        % split into regions to obtain single-regions obstacles
        reg = regions(initialstate_polyshape);

        if length(reg) > 1
            fprintf('More than one region in shallow.\n')
        end

        for l = 1:size(reg,1)
            shallows_counter = shallows_counter + 1;
            vert = reg(l,1).Vertices;
            shallows{end+1,1} = polygon(vert(:,1),vert(:,2));
        end
    end
    
        
    % -----------------------------------------------
    % -- Extract all Ego Vehicles -------------------
    % -----------------------------------------------
    
    num_goalRegions = 0;
    num_planningProblems = 0;
    
    egoVehiclesList = xDoc.getElementsByTagName('planningProblem');
    if ~isempty(egoVehiclesList.item(0))
        num_planningProblems = egoVehiclesList.getLength();
        x0(num_planningProblems).x = [];
        goalSet = cell(1,num_planningProblems);
        
        for i = 0:(num_planningProblems-1)
            num_goalRegions(i+1) = 0;
            
            % initial state
            InitialStateList = egoVehiclesList.item(i).getElementsByTagName('initialState');
            x0(i+1).x = getXEl(egoVehiclesList, {'initialState', 'position', 'point', 'x'}, i);
            x0(i+1).y = getXEl(egoVehiclesList, {'initialState', 'position', 'point', 'y'}, i);
            
            % retrieval of x0-struct information
            timeList_initial = getXList(InitialStateList, {'time'});
            x0(i+1).time = interval_or_exact(timeList_initial);
            x0(i+1).velocity = getXEl(InitialStateList, {'velocity', 'exact'});
            %%%%%%% WHAT IF THE MODELLING VARIABLES ARE DIFFERENT?
            x0(i+1).orientation = getXEl(InitialStateList, {'orientation', 'exact'});
            
            % goal region
            goalStateList = getXList(egoVehiclesList, {'goalState'}, i);
            for k = 0:(goalStateList.getLength()-1)
                num_goalRegions(i+1) = num_goalRegions(i+1) + 1;
                
                % get position/goal space
                positionList = getXList(goalStateList, {'position'}, k);
                assert(positionList.getLength() < 2, 'Violation of standard: More than one position for goalState.\n')
                if ~isempty(positionList.item(0))
                    
                    % waterways
                    watersList = positionList.item(0).getElementsByTagName('waterway');
                    waters_length = watersList.getLength;
                    polyshapeBuffer = polyshape();
                    for m = 0:(waters_length-1)
                        ref = str2double(watersList.item(m).getAttribute('ref'));
                        idx = [generalWatersList.id] == ref;
                        polyshapeBuffer = union(polyshapeBuffer,...
                            polyshape([generalWatersList(idx).leftBound, flip(generalWatersList(idx).rightBound, 2)]'));
                    end
                    
                    % region by lanelets if found above
                    if waters_length
                        goalSet{num_goalRegions(i+1),i+1}.set =...
                            polygon(polyshapeBuffer.Vertices(:,1), polyshapeBuffer.Vertices(:,2));
                    
                    % region by shape
                    else
                        verticesBuffer = shape_fct(positionList);
                        goalSet{num_goalRegions(i+1),i+1}.set =...
                            polygon(verticesBuffer(:,1), verticesBuffer(:,2));
                    end
                else
                    % insert empty set if no goal set found
                    goalSet{num_goalRegions(i+1),i+1}.set = [];
                end
                
                % time
                timeList_goal = getXList(goalStateList, {'time'}, k);
                goalSet{i+1}.time = interval_or_exact(timeList_goal)*timeStep;
                
                % velocity
                velList_goal = getXList(goalStateList, {'velocity'}, k);
                if ~isempty(velList_goal.item(0))
                    goalSet{i+1}.velocity = interval_or_exact(velList_goal);
                else
                    goalSet{i+1}.velocity = [];
                end
                
                % orientation (Note: Additional measures have to be taken to
                % avoid confusion between shape orientation and state
                % orientation in xml
                orientList_goal = getXList(goalStateList, {'orientation'}, k);
                if ~isempty(orientList_goal.item(0))
                    for l = 0:(orientList_goal.getLength()-1)
                        if strcmp('goalState', orientList_goal.item(l).getParentNode.getNodeName)
                            goalSet{i+1}.orientation = interval_or_exact(orientList_goal, l);
                            break
                        end
                    end
                else
                    goalSet{i+1}.orientation = [];
                end
            end
        end
    else
        % Return empty arrays if no planning problem exists
        if verbose
            %fprintf('No planning problem found. Will return empty vectors x0, goalSet');
        end
        x0 = [];
        goalSet = [];
    end
    
    % -----------------------------------------------
    % -- Extract further information ----------------
    % -----------------------------------------------
    
    if strcmp(commonocean_version, '2022a')
        information = struct();
        information.BadRes = counterBadOnes;
        information.timeStep = timeStep;
        try
            
            % tags
            tags_all = xDoc.getElementsByTagName('scenarioTags').item(0).getElementsByTagName('*');
            tags = cell(length(tags_all));
            
            for i = 0:(length(tags_all)-1)
                tags{i+1} = string(tags_all.item(i).getTagName);
            end
            information.tags = tags;

            % location
            information.location.geoNameId = str2double(xDoc.getElementsByTagName('location').item(0)...
                .getElementsByTagName('geoNameId').item(0).getTextContent);
            information.location.gpsLatitude = str2double(xDoc.getElementsByTagName('location').item(0)...
                .getElementsByTagName('gpsLatitude').item(0).getTextContent);
            information.location.gpsLongitude = str2double(xDoc.getElementsByTagName('location').item(0)...
                .getElementsByTagName('gpsLongitude').item(0).getTextContent);

        end
    end
    
    % -----------------------------------------------
    % -- Transform to required output format --------
    % -----------------------------------------------
    
    % --- Static Obstacles unit test---
    assert(length(statObs) == static_counter, 'Counter mismatch for static obstacles.')
    
    % --- Dynamic Obstacles unit test ---
    assert(length(dynObs) == total_dynamic_counter, 'Counter mismatch for dynamic obstacles.')
    
    % --- Check for multiple Planning Problems and reduce to one ---
    if num_planningProblems > 1
        if verbose
            fprintf(strcat('There are several planning problems found. However, only one',...
                'can be handled by the converter, thus only the first one will be regarded.\n'));
        end
        x0 = x0(1);
        goalSet = goalSet(:,1);
    end
    
    warning(w);
    converterTime = toc(conTimerStart);
    
    if verbose
        fprintf(strcat('--------------\nSuccessfully converted\n', filename,...
            ' with\n%d waterways,\n%d shallows,\n',...
            '%d static obstacles,\n%d dynamic obstacles,\n',...
            '%d planning problem with\n%d goal regions in\n%.2f seconds.\n--------------\n'),...
            length(waterways),shallows_counter, static_counter, dynamic_counter, egoVehiclesList.getLength, num_goalRegions, converterTime);
    end

end

%------------- AUXILIARY FUNCTIONS --------------%

function vertices = shape_fct(shapeList, stateXList, idx)
% This function returns vertices for the combined polyshape which is the
% union of all shapes contained in shapeList, possibly combined with
% position and orientation from the optional parameter stateXList. You can
% choose which states from stateXList with index idx are relevant by
% passing the optional parameter idx.

if ~exist('idx', 'var')
    idx = 0;
end

if ~exist('stateXList', 'var')
    orientation_state = 0;
    x_state = 0;
    y_state = 0;
else
    assert(stateXList.getLength() >= idx + 1,...
        sprintf('Expected state list of min length %i; got length %i.\n', idx, stateXList.getLength()));
    orientation_state = getXEl(stateXList, {'orientation', 'exact'}, idx);
    x_state = getXEl(stateXList, {'position', 'point', 'x'}, idx);
    y_state = getXEl(stateXList, {'position', 'point', 'y'}, idx);
end

% initialize empty shape
shape_polygon = polyshape();

% rectangle
rectangleList = getXList(shapeList, {'rectangle'});
for l = 0:(rectangleList.getLength()-1)
    len = getXEl(rectangleList, {'length'}, l);
    wid = getXEl(rectangleList, {'width'}, l);
    
    if ~isempty(rectangleList.item(l).getElementsByTagName('center').item(0))
        x_center = getXEl(rectangleList, {'center', 'x'}) + x_state;
        y_center = getXEl(rectangleList, {'center', 'y'}) + y_state;
    else
        x_center = x_state;
        y_center = y_state;
    end
    
    orientationXList = getXList(rectangleList, {'orientation'}, l);
    if ~isempty(orientationXList.item(0))
        if isempty(orientationXList.item(0).getElementsByTagName('exact').item(0))
            orientation = getXEl(orientationXList) + orientation_state;
        else
            orientation = getXEl(orientationXList, {'exact'}) + orientation_state;
        end
    else
        orientation = orientation_state;
    end
    
    points = [-len, -wid; len, -wid; len, wid; -len, wid]/2;
    points = points * [cos(orientation), sin(orientation); -sin(orientation), cos(orientation)] + [x_center, y_center];
    
    % add all the rectangles(polygons)
    shape_polygon = union(shape_polygon, polyshape(points,'Simplify',false));
end

% circle
circleList = getXList(shapeList, {'circle'});
for l = 0:(circleList.getLength()-1)
    radius = getXEl(circleList, {'radius'}, l);
    
    if ~isempty(circleList.item(0).getElementsByTagName('center').item(0))
        x_center = getXEl(circleList, {'center', 'x'}) + x_state;
        y_center = getXEl(circleList, {'center', 'y'}) + y_state;
    else
        x_center = x_state;
        y_center = y_state;
    end 
    
    % convering circle into a polygon
    n = 50; % number of points approximating the circle
    theta = (0:n-1)*(2*pi/n);
    x = x_center + radius*cos(theta);
    y = y_center + radius*sin(theta);
    
    % add all the circles(polygons)
    shape_polygon = union(shape_polygon, polyshape(x,y,'Simplify',false));
end

% polygon
polygonList = getXList(shapeList, {'polygon'});
for l = 0:(polygonList.getLength()-1)
    polygon_pointList = getXList(polygonList, {'point'}, l);
    points = zeros(polygon_pointList.getLength(), 2);
    %y = zeros(1,polygon_pointList.getLength());
    for k = 0:(polygon_pointList.getLength()-1)
        points((k+1), 1) = getXEl(polygon_pointList, {'x'}, k);
        points((k+1), 2) = getXEl(polygon_pointList, {'y'}, k);
    end

    % add all the polyshapes
    shape_polygon = union(shape_polygon, polyshape(points,'Simplify',false));
end

vertices =  shape_polygon.Vertices;
end

function dynamicBuffer = dynamic_fct(obstaclesList, i)
% This function returns a list of dynamic Obstacles extracted from
% obstaclesList.item(i). It handles occupancySets and trajectories.

% initial state
ShapeList_initialstate = getXList(obstaclesList, {'shape'}, i);
stateList_initialstate = getXList(obstaclesList, {'initialState'}, i);
initialstate_vertices = shape_fct(ShapeList_initialstate, stateList_initialstate);

occupancySetXList = getXList(obstaclesList, {'occupancySet'}, i);
trajectoryList = getXList(obstaclesList, {'trajectory'}, i);

% by occupancySet
if ~isempty(occupancySetXList.item(0))
    occupancyList = getXList(occupancySetXList, {'occupancy'});
    
    % creating obstacles
    dynamicBuffer = occupancy_fct(occupancyList);
    
% by trajectory
elseif ~isempty(trajectoryList.item(0))
    trajectorystateList = getXList(trajectoryList, {'state'});
    
    % creating obstacles
    dynamicBuffer = trajectory_fct(trajectorystateList, ShapeList_initialstate);
else
    fprintf(strcat('Found dynamic obstacle without dynamics or with probability',...
        'distribution, which cannot be handled by converter. Resuming...\n'));
end

% inserting the initial state
dynamicBuffer(1).polygon = initialstate_vertices;
dynamicBuffer(1).time = 0;

end

function obsmat = dynamic_mat(obstaclesList, timeStepSize, i)
% This function returns a list of dynamic Obstacles extracted from
% obstaclesList.item(i). It handles occupancySets and trajectories.

% initial state
initialTimeStep = zeros(1,7);
stateList_initialstate = getXList(obstaclesList, {'initialState'}, i);
initialTimeStep(1,1) = i+1;
initialTimeStep(1,2) = getXEl(stateList_initialstate, {'position', 'point', 'x'}); %x_position
initialTimeStep(1,3) = getXEl(stateList_initialstate, {'position', 'point', 'y'}); %y_position
initialTimeStep(1,4) = getXEl(stateList_initialstate, {'orientation', 'exact'}); %orientation
initialTimeStep(1,5) = getXEl(stateList_initialstate, {'velocity', 'exact'}); %velocity

trajectoryList = getXList(obstaclesList, {'trajectory'}, i);
% by trajectory
if ~isempty(trajectoryList.item(0))
    trajectorystateList = getXList(trajectoryList, {'state'});
    trajectorystateListLength = trajectorystateList.getLength;

    obsmat = zeros((trajectorystateListLength + 1),7);
    obsmat(1,:) = initialTimeStep;
    
    if ~isempty(trajectorystateList.item(0))
        for j = 0 : (trajectorystateListLength -1)
            % if mod(j, 6) == 0
                obsmat(j+2,1) = i+1;
    
                midval = getXList(trajectorystateList, {'position', 'point', 'x'}, j); %x_position
                obsmat(j+2,2) = getXEl(midval); %x_position
    
                obsmat(j+2,3) = getXEl(trajectorystateList, {'position', 'point', 'y'}, j); %y_position
                obsmat(j+2,4) = getXEl(trajectorystateList, {'orientation', 'exact'}, j); %orientation
                obsmat(j+2,5) = getXEl(trajectorystateList, {'velocity', 'exact'}, j); %velocity
        
                % Calculate Velocities of all points
                obsmat(j+1,6) = (obsmat(j+2,2) - obsmat(j+1,2)) / timeStepSize; %velocity x
                obsmat(j+1,7) = (obsmat(j+2,3) - obsmat(j+1,3)) / timeStepSize; %velocity y
            % end
        end
    end

    obsmat(:,1) = i+1;
    % Remove last data point as it has velocity = 0
    obsmat(size(obsmat),:) = [];

else
    fprintf(strcat('Found dynamic obstacle without dynamics or with probability',...
        'distribution, which cannot be handled by converter. Resuming...\n'));
end

end

function occupancy = occupancy_fct(occupancyList)
% This function returns a list of occupancy structs for a given xDocList
occupancy(occupancyList.getLength).polygon = [];
for l = 0:(occupancyList.getLength()-1)
    
    % polygon
    shapeList_occupancy = getXList(occupancyList, {'shape'}, l);
    if ~isempty(shapeList_occupancy.item(0))
        occupancy(l+2).polygon = shape_fct(shapeList_occupancy);
    else
        assert(0, 'This is a temporary developer warning indicating unexpected behaviour: No shapes found in occupancy.')
    end
    
    % time
    timeList_occupancy = getXList(occupancyList, {'time'}, l);
    if ~isempty(timeList_occupancy.item(0).getElementsByTagName('exact').item(0))
        occupancy(l+2).time = getXEl(timeList_occupancy, {'exact'});
        
    elseif ~isempty(timeList_occupancy.item(0).getElementsByTagName('intervalStart').item(0))
        occupancy(l+2).time(1) = getXEl(timeList_occupancy, {'intervalStart'});
        occupancy(l+2).time(2) = getXEl(timeList_occupancy, {'intervalEnd'});
    end
end

end

function trajectory = trajectory_fct(trajectoryStateXList, shapeXList_initialstate)
% This function returns a list of trajectory polygon structs for a given xDocList

trajectory(trajectoryStateXList.getLength).polygon = [];
for l = 0:(trajectoryStateXList.getLength()-1)
    % polygon
    trajectory(l+2).polygon = shape_fct(shapeXList_initialstate, trajectoryStateXList, l);
    
    % time
    timeList = getXList(trajectoryStateXList, {'time', 'exact'}, l);

    if ~isempty(timeList.item(0))
        trajectory(l+2).time = getXEl(timeList);
    end
end
end


function elements = getXEl(xDocList, strings, indexes)
% Returns elements queried in cell array strings. If no vector of indexes
% is provided, the first element (0-element) is returned. Indexes in
% indexes are taken in order, starting with the first element until no
% further elements are provided. There may be more elements in strings than
% in indexes.

if ~exist('strings', 'var')
    strings = [];
end

if ~exist('indexes', 'var') || isempty(indexes)
    indexes = 0;
end

if ~isempty(strings)
    elements = getXEl(xDocList.item(indexes(1)).getElementsByTagName(strings{1}), strings(2:end), indexes(2:end));
else
    assert(length(indexes) == 1, 'wrong number of indexes provided')
    assert(xDocList.getLength() == 1, 'Several items found, where one was expected.')
    
    elements = str2double(xDocList.item(indexes).getTextContent);
end

end

function elements = getXList(xDocList, strings, indexes)
% Returns elements queried in cell array strings. If no vector of indexes
% is provided, the first element (0-element) is returned. Indexes in
% indexes are taken in order, starting with the first element until no
% further elements are provided. There may be more elements in strings than
% in indexes.

if ~exist('indexes', 'var') || isempty(indexes)
    indexes = 0;
end

if length(strings) > 1
    elements = getXList(xDocList.item(indexes(1)).getElementsByTagName(strings{1}), strings(2:end), indexes(2:end));
else
    elements = xDocList.item(indexes(1)).getElementsByTagName(strings{1});
end

end

function inter_or_exact = interval_or_exact(XList, idx)
% This function returns either an interval of two values or a single value
% from item(idx) given in XList, depending on whether the childNodes are
% 'exact' or 'intervalStart'/'intervalEnd'

if ~exist('idx', 'var')
    idx = 0;
end

if ~isempty(XList.item(idx).getElementsByTagName('exact').item(0))
    inter_or_exact = getXEl(XList, {'exact'}, idx);
elseif ~isempty(XList.item(idx).getElementsByTagName('intervalStart').item(0))
    inter_or_exact = interval(getXEl(XList, {'intervalStart'}, idx), getXEl(XList, {'intervalEnd'}, idx));
end

end