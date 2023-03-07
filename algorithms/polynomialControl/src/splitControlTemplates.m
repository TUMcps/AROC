function list = splitControlTemplates(Uscalar,ids,splits)
% SPLITCONTROLTEMPLATES - split poly. zonotopes for controller templates
%
% Syntax:
%       list = SPLITCONTROLTEMPLATES(Uscalar,ids,splits)
%
% Description:
%       Recursively split the polynomial zonotopes for the controller 
%       templates to improve the accuracy of the bounds that characterize 
%       feasible parameters.
%
% Input Arguments:
%
%       -Uscalar:   controller template for a scalar control input
%       -ids:       identifiers for the factors that are split
%       -splits:    number of recursive splits
%
% Output Arguments:
%
%       -list:      list storing the split controller templates
%
% See Also:
%       polynomialControl
%
% References:
%       * *[1] Gassman et al. (2021)*, Verified Polynomial Controller 
%              Synthesis for Disturbed Nonlinear Systems
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % determine identifiers of factors that are split
    ind = ~ismember(Uscalar.id,ids);
    id = Uscalar.id(ind);
    nw = length(id);
    
    % polynomial zonotopes after splitting a single factor
    pZ1 = polyZonotope(1/2,1/2,[],1);
    pZ2 = polyZonotope(-1/2,1/2,[],1);

    % loop over the number of recursive splits
    list{1} = Uscalar;

    for i = 1:splits

        % select the factor that is splitted
        ii_split = mod(i,nw);
        if ~ii_split
            ii_split = nw;
        end
        id_split = id(ii_split);

        % split all polynomial zonotopes in the current list
        list_new = cell(2*length(list),1);

        for j = 1:length(list)
            pZtmp = list{j};
            pZnew1 = subs(pZtmp,replaceId(pZ1,id_split),id_split);
            pZnew2 = subs(pZtmp,replaceId(pZ2,id_split),id_split);
            list_new{(j-1)*2+1} = pZnew1;
            list_new{(j-1)*2+2} = pZnew2;
        end

        list = list_new;
    end

    % split the resulting polynomial zonotopes into the single generators
    list_tmp = cell(length(list),1);

    for i = 1:length(list)
        [~,tmp] = partZonotope(list{i},id);
        list_tmp{i} = tmp(:);
    end

    list = list_tmp;