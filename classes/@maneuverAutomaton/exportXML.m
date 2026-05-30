function exportXML(obj,file,varargin)
% EXPORTXML - Store a maneuver automaton in a .xml file
%
% Syntax:
%       EXPORTXML(obj,file)
%       EXPORTXML(obj,file,states)
%       EXPORTXML(obj,file,states,inputs)
%
% Description:
%       This function creates a .zip folder that contains an .xml file 
%       storing the maneuver automaton as well as .mat files that contain 
%       the data referenced by the .xml file.
%
% Input Arguments:
%
%       -obj:       maneuverAutomaton object
%       -file:      name of the file in which the maneuver automaton is
%                   saved
%       -states:    cell-array containing the names of the states of the
%                   system (e.g. {'x','y','velocity','orientation'})
%       -inputs:    cell-array containing the names of the inputs of the
%                   system (e.g. {'acceleration','steer'})
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% parse input arguments
nx = dim(obj.primitives{1}.R0);
statenames = num2cell(string(sym('x',[nx,1])));

if nargin > 2 && ~isempty(varargin{1})
    if ~iscell(varargin{1}) || length(varargin{1}) < nx
        error('Wrong value for input arugment "states"!');
    end
    statenames = varargin{1};
end

nu = dim(obj.primitives{1}.U);
inputnames = num2cell(string(sym('u',[nu,1])));

if nargin > 3 && ~isempty(varargin{2})
    if ~iscell(varargin{2}) || length(varargin{2}) < nu
        error('Wrong value for input arugment "inputs"!');
    end
    inputnames = varargin{2};
end

docNode = com.mathworks.xml.XMLUtils.createDocument('maneuverAutomaton');
maneuverAutomaton = docNode.getDocumentElement;

% create directory for storing the temporary files
[path,~,~] = fileparts(which(mfilename));
path = fullfile(path,'temp');
mkdir(path);
pathPrim = fullfile(path,'primitives');
mkdir(pathPrim);
pathContr = fullfile(path,'controllers');
mkdir(pathContr);

% loop over all motion primitives
for i = 1:length(obj.primitives)

    % create motion primitive xml-object
    motionPrimitive = docNode.createElement('motionPrimitive');
    primitivefile = ['primitive',num2str(i),'.xml'];
    motionPrimitive.appendChild(docNode.createTextNode(primitivefile));

    motionPrimitive.setAttribute('ID',num2str(i));
    maneuverAutomaton.appendChild(motionPrimitive);

    % create the .xml file for the motion primitive
    exportMotionPrimitiveXML(obj,i,inputnames,statenames,pathPrim)
end 

% create .xml file
fileXML = fullfile(path,'automaton.xml');
xmlwrite(fileXML,maneuverAutomaton);

% create controller files
excluded = {'nx','nu','nw','xf','tFinal','R0','Rfin','V','W', ...
                                            'dynamics','occupancySet'};

w = warning();
warning('off');

for i = 1:length(obj.primitives)
    prop = properties(obj.primitives{i});
    args = {};
    for j = 1:length(prop)
        if ~ismember(prop{j},excluded)
            eval([prop{j},' = convert(obj.primitives{i}.',prop{j},');']);
            args{end+1} = prop{j};
        end
    end
    save(fullfile(pathContr,['controller',num2str(i),'.mat']),args{:});
end

warning(w);

% created .zip file storing the maneuver automaton
[p,f,~] = fileparts(file);
file = fullfile(p,f);

zip(file,{fileXML,pathContr,pathPrim});

% delete temporary folder
rmdir(path,'s');

end


% Auxiliary Functions -----------------------------------------------------

function exportMotionPrimitiveXML(obj,i,inputnames,statenames,path)
% export a single motion primitive to a .xml file

    nx = dim(obj.primitives{1}.R0);
    nu = dim(obj.primitives{1}.U);

    % create motion primitive xml-object
    docNode = com.mathworks.xml.XMLUtils.createDocument('motionPrimitive');
    motionPrimitive = docNode.getDocumentElement;
    motionPrimitive.setAttribute('ID',num2str(i));

    % simulate system to get reference trajectory
    [~,t_,x_,u_] = simulate(obj,i,center(obj.primitives{i}.R0));

    t = cellfun(@(x) supremum(x.time),obj.primitives{i}.occupancySet);
    t = [t;cellfun(@(x) infimum(x.time),obj.primitives{i}.occupancySet)];
    t = unique(t);

    [~,ind] = unique(t_);
    x = interp1(t_(ind),x_(ind,:),t,'linear','extrap');
    u = interp1(t_(ind),u_(ind,:),t,'linear','extrap');

    % add controller xml-element
    controller = docNode.createElement('controller');
    motionPrimitive.appendChild(controller);

    name = '';

    if isa(obj.primitives{i},'objCombinedContr')
        name = 'combined control';
    elseif isa(obj.primitives{i},'objConvInterContr')
        name = 'convex interpolation control';
    elseif isa(obj.primitives{i},'objGenSpaceContr')
        name = 'generator space control';
    elseif isa(obj.primitives{i},'objOptBasedContr')
        name = 'optimization based control';
    elseif isa(obj.primitives{i},'objPolyContr')
        name = 'polynomial control';
    elseif isa(obj.primitives{i},'objSafetyNetContr')
        name = 'safety net control';
    end

    controller.setAttribute('type',name);
    controllerfile = ['controller',num2str(i),'.mat'];
    controller.appendChild(docNode.createTextNode(controllerfile));
    
    % add successor xml-element
    successors = docNode.createElement('successors');
    motionPrimitive.appendChild(successors);
    
    for j = 1:length(obj.primitives)
        if obj.conMat(i,j) == 1
            id = docNode.createElement('id');
            successors.appendChild(id);
            id.appendChild(docNode.createTextNode(num2str(j)));
        end
    end

    % add trajectory xml-object
    traj = docNode.createElement('trajectory');
    motionPrimitive.appendChild(traj);

    % add states element
    states = docNode.createElement('states');
    traj.appendChild(states);
    
    for j = 1:length(t)
    
        state = docNode.createElement('state');
        states.appendChild(state);

        % add time 
        time = docNode.createElement('time');
        state.appendChild(time);
        time.appendChild(docNode.createTextNode(num2str(t(j))));

        % add states
        for k = 1:nx
            x_node = docNode.createElement(statenames{k});
            state.appendChild(x_node);
            x_node.appendChild(docNode.createTextNode(num2str(x(j,k))));
        end            
    end

    % add inputs
    inputs = docNode.createElement('inputs');
    traj.appendChild(inputs);
    
    for j = 1:length(t)-1
    
        input = docNode.createElement('input');
        inputs.appendChild(input);

        for k = 1:nu
            u_node = docNode.createElement(inputnames{k});
            input.appendChild(u_node);
            u_node.appendChild(docNode.createTextNode(num2str(u(j,k))));
        end            
    end
    
    % add occupancy xml-element
    occSet = docNode.createElement('occupancySet');
    motionPrimitive.appendChild(occSet);
    
    for j = 1:length(obj.primitives{i}.occupancySet)
        
        node_occ = docNode.createElement('occupancy');
        occSet.appendChild(node_occ);

        % add time
        time = docNode.createElement('time');
        node_occ.appendChild(time);

        node_start = docNode.createElement('intervalStart');
        time.appendChild(node_start);
        time_start = infimum(obj.primitives{i}.occupancySet{j}.time);
        node_start.appendChild(docNode.createTextNode(num2str(time_start)));

        node_end = docNode.createElement('intervalEnd');
        time.appendChild(node_end);
        time_end = supremum(obj.primitives{i}.occupancySet{j}.time);
        node_end.appendChild(docNode.createTextNode(num2str(time_end)));

        % add set
        node_set = docNode.createElement('set');
        node_occ.appendChild(node_set);
        set = obj.primitives{i}.occupancySet{j}.set;
        
        if isa(set,'polygon')
            V = set.set.Vertices';
        else
            V = vertices(set);
        end
        
        % loop over all vertices of the set
        for k = 1:size(V,2)

            node_point = docNode.createElement('point');
            node_set.appendChild(node_point);

            % loop over all dimensions
            for l = 1:size(V,1)

                element = docNode.createElement('element');
                element.setAttribute('dimension',num2str(l));
                element.appendChild(docNode.createTextNode(num2str(V(l,k))));
                node_point.appendChild(element);
            end
        end
    end

    % add reachability xml-element
    node_reachSet = docNode.createElement('reachableSet');
    motionPrimitive.appendChild(node_reachSet);
    
    R = {obj.primitives{i}.R0; obj.primitives{i}.Rfin};
    t = {t(1); t(end)};

    for j = 1:length(R)

        node_reach = docNode.createElement('reachability');
        node_reachSet.appendChild(node_reach);

        % add time
        node_time = docNode.createElement('time');
        node_reach.appendChild(node_time);
        node_time.appendChild(docNode.createTextNode(num2str(t{j})));
        
        % add set
        set = interval(R{j});
        
        for k  = 1:nx

            node_elem = docNode.createElement(statenames{k});
            node_reach.appendChild(node_elem);

            node_start = docNode.createElement('intervalStart');
            node_elem.appendChild(node_start);
            node_start.appendChild(docNode.createTextNode(num2str(infimum(set(k)))));

            node_end = docNode.createElement('intervalEnd');          
            node_elem.appendChild(node_end);
            node_end.appendChild(docNode.createTextNode(num2str(supremum(set(k)))));
        end                  
    end

    % create .xml file
    fileXML = fullfile(path,['primitive',num2str(i),'.xml']);
    xmlwrite(fileXML,motionPrimitive);
end

function obj = convert(obj)
% recursive function to bring data objects to an exportable format

    if isnumeric(obj)
        return;
    elseif iscell(obj)
        for i = 1:size(obj,1)
            for j = 1:size(obj,2)
                obj{i,j} = convert(obj{i,j});
            end
        end
    else
        obj = struct(obj);
    end
end