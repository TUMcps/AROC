function ind = motionPlanner(obj,x0,goalSet,statObs,dynObs,search)
% MOTIONPLANNER - plan a verified trajectoy with a Maneuver Automaton
%
% Syntax:
%       ind = MOTIONPLANNER(obj,x0,goalSet,statObs,dynObs,search)
%
% Description:
%       This function solves a control task by planning a verified
%       trajectoy with a Maneuver Automaton. In particular, the verified
%       trajectory avoids the static and dynamic obstacles and drives the
%       system to the goal set.
%
% Input Arguments:
%
%       -obj:       Maneuver automaton (class: maneuverAutomaton)
%       -x0:        initial state
%       -goalSet:   goal set which should be reached
%       -statObs:   cell-array storing the static obstacles
%       -dynObs:    cell-array storing the dynamic obstacles
%       -search:    search algorithm to be used ('breadth-first', ...
%                   'depth-first', or 'Astar')
%
% Output Arguments:
%
%       -ind:       vector storing the indices of the motion primitives
%                   that are concatanated to obtain the verified trajectory
%
% See Also:
%       maneuverAuotomaton
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
% Authors:      Jinyue Guan, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % initialize the frontier with all motion primitive that are compatible
    % with the initial state x0
    frontier = initializeFrontier(obj,x0);

    % search until a valid solution is found or the frontier is empty
    while ~isempty(frontier)

        % select the next node depending on the used search algorithm
        if strcmp(search,'breadth-first')
            node = frontier{1};
            frontier = frontier(2:end);
        elseif strcmp(search,'depth-first')
            node = frontier{end};
            frontier = frontier(1:end-1);
        elseif strcmp(search,'Astar')
            frontier = sortFrontier(obj,frontier,goalSet);
            node = frontier{end};
            frontier = frontier(1:end-1);
        else
            error('Wrong value for input argument "search"!'); 
        end

        % compute the occupancy set for the current node using the final
        % state and time from its parent node
        index = node.ind(end);
        x = obj.updateState(node.parent.xf,index); 
        occSet = updateOccupancy(obj,node.parent.xf,index,node.parent.time);
        time = supremum(occSet{end}.time);

        % check if the occupancy set collides with obstacles
        if collisionChecker(occSet,statObs,dynObs)

            % check if the goal set is reached 
            if time >= infimum(goalSet.time)
                for i = 1:length(occSet)
                    if in(goalSet.time,occSet{i}.time)
                        if in(goalSet.set,occSet{i}.set)
                            ind = node.ind;
                            return
                        end
                    end               
                end
            end

            % calculate child nodes and add them to the frontier
            if supremum(occSet{end}.time) <= supremum(goalSet.time)
                newNodes = constructChildNodes(obj,node,x,time);
                frontier = [frontier;newNodes];
            end
        end
    end
    
    % throw error if the search failed
    error('Search failed!');
end


% Auxiliary Functions -----------------------------------------------------
    
function frontier = initializeFrontier(obj,x0)
% initialize the frontier with all motion primitive that are compatible
% with the initial state x0

    frontier = {};
    counter = 1;

    % loop over all motion primitives
    for i = 1:length(obj.primitives)
        
        % check if the initial set of the motion primitive contains the
        % initial point x0
        Rinit = obj.primitives{i}.R0;
        Rinit = obj.shiftFun(Rinit,x0);
        
        if in(Rinit,x0)
            
            % add the motion primitive to the frontier
            frontier{counter,1}.ind = i;
            frontier{counter,1}.parent.xf = x0;
            frontier{counter,1}.parent.time = 0;
            
            counter = counter + 1;
        end       
    end
end

function newNodes = constructChildNodes(obj,node,x,time)
% construct all child nodes for the current node

    % get all motion-primitives that can be connected to the current motion
    % primitive
    ind = find(obj.conMat(node.ind(end),:) == 1);
    newNodes = cell(length(ind),1);
    
    % loop over all child-nodes
    for i = 1:length(ind)
       newNodes{i}.ind = [node.ind;ind(i)];
       newNodes{i}.parent.xf = x;
       newNodes{i}.parent.time = time;
    end
end

function frontier = sortFrontier(obj,frontier,goalSet)
% sort the frontier according to the costs

    % compute the costs for all nodes
    costs = zeros(length(frontier),1);
    
    for i = 1:length(frontier)
       if ~isfield(frontier{i},'costs')
           frontier{i}.costs = computeCosts(obj,frontier{i},goalSet);
       end
       costs(i) = frontier{i}.costs;
    end
    
    % sort the frontier according to the costs
    [~,ind] = sort(costs,'ascend');
    frontier = frontier(ind);
end

function cost = computeCosts(obj,node,goalSet)
% compute the costs for A* star search for the current node

    % compute final state and final time at the end of the motion primitive
    index = node.ind(end);
    occSet = updateOccupancy(obj,node.parent.xf,index,node.parent.time);
    xCurr = center(occSet{end}.set);
    time = node.parent.time + obj.primitives{index}.tFinal;
    
    % compute estimated remaining time for reaching the goal set
    v = 30;
    c = center(goalSet.set);
    dist = sqrt(sum(c-xCurr).^2);
    h = dist/v;
    g = time;
    
    % compute costs for the node
    cost = h + g;
end