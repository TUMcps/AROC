function checkAROCdependencies()
%CHECKDEPENDENCIES  - check whether the required thrid-party toolboxes are
%installed and correctness of the version.
%
% Syntax:
%       checkAROCdependencies()
%
% Description:
%       This function checks the availability of all required third-party
%       toolboxes and reports their installed and recommended versions.
%
%       The following dependencies are checked:
%           - CORA
%           - MOSEK
%           - YALMIP
%           - CasADi
%
% See Also:
%       -
% 
% References:
%       -
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2026 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% ------ recommended version ---------------------------------------------

desired.CORA   = 'v2026.0.1';
desired.MOSEK  = '11.1';
desired.YALMIP = '20250626';
desired.CasADi = '3.7.2';

% ------ initialize ------------------------------------------------------

tools      = {};
installed  = {};
version    = {};
desiredVer = {};

% ------ CORA ------------------------------------------------------------
try
    if exist(sprintf('%s/contDynamics',CORAROOT),'dir') && ...
        exist(sprintf('%s/contSet',CORAROOT),'dir') 
        coraInstalled = true;
    else
        coraInstalled = false;
    end
catch
    coraInstalled = false;
end

tools{end+1}      = 'CORA';
installed{end+1}  = yesno(coraInstalled);
version{end+1}    = 'Unknown';
desiredVer{end+1} = desired.CORA;

% ------ YALMIP ----------------------------------------------------------

try
    yalmipVersion = char(yalmip('version'));
    yalmipInstalled = true;

catch
    yalmipInstalled = false;
    yalmipVersion   = 'Not found';
end

tools{end+1}      = 'YALMIP';
installed{end+1}  = yesno(yalmipInstalled);
version{end+1}    = yalmipVersion;
desiredVer{end+1} = desired.YALMIP;

% ------ MOSEK -----------------------------------------------------------
try
    evalc('[~,res] = mosekopt(''version'')');
    mosekInstalled = true;
    if isstruct(res) && isfield(res,'version') && ...
            isfield(res.version,'major') && isfield(res.version,'minor')
        mosekVersion = [num2str(res.version.major),'.',num2str(res.version.minor)];
    else
        mosekVersion = 'Unknown';
    end
catch
    mosekInstalled = false;
    mosekVersion = 'Not found';
end

tools{end+1}      = 'MOSEK';
installed{end+1}  = yesno(mosekInstalled);
version{end+1}    = mosekVersion;
desiredVer{end+1} = desired.MOSEK;

% ------ CasADi-----------------------------------------------------------

try
    casadiVersion = casadi.CasadiMeta.version();
    casadiInstalled = true;
catch
    casadiInstalled = false;
    casadiVersion = 'Not found';
end

tools{end+1}      = 'CASADI';
installed{end+1}  = yesno(casadiInstalled);
version{end+1}    = casadiVersion;
desiredVer{end+1} = desired.CasADi;

% ------ show results ----------------------------------------------------

report = table( ...
    categorical(tools(:)), ...
    categorical(installed(:)), ...
    categorical(version(:)), ...
    categorical(desiredVer(:)), ...
    'VariableNames', { ...
        'Tool', ...
        'Installed', ...
        'Installed Version', ...
        'Recommended Version'} ...
        );

disp(report);

end

%% auxiliary functions

function out = yesno(x)

if x
    out = 'YES';
else
    out = 'NO';
end

end