function res = resolveControlTemplates(P,c,Opts)
% RESOLVECONTROLTEMPLATES - update templates for the control parameters
%
% Syntax:
%       res = RESOLVECONTROLTEMPLATES(P,c,Opts)
%
% Description:
%       Replace certain factors in the polynomial zonotopes that represent 
%       the templates for the control parameters by concrete values.
%
% Input Arguments:
%
%       -P:     list storing the templates for the control parameters
%       -c:     values for the factors that are replaced
%       -Opts:  structure containing all options
%
% Output Arguments:
%
%       -res:   struct storing the updated templates
%
% See Also:
%       polynomialControl
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

    % initialization
    Ninter = Opts.Ninter; nu = Opts.nu;
    len = Opts.extHorizon.length;
    steps = Ninter*len;
    
    % loop over all splitted parameter sets
    list = cell(length(P),1);

    for i = 1:length(P)

        Ptmp = zeros(length(P{i}),length(Opts.idp_s));

        for j = 1:length(P{i})

            % replace constant parameters
            pZtmp = resolve(P{i}{j},c,Opts.idc);

            if isZero(pZtmp)
                continue;
            end

            % compute coefficients (template is always linear in p)
            Ptmp(j,:) = pZtmp.G*pZtmp.expMat';
        end

        % do not remove zero row!
        list{i} = Ptmp;
    end

    % store output arguments
    res.list = list;
    res.steps = steps;
    res.nu = nu;
end