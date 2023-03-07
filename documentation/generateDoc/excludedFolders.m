function folders = excludedFolders()
% Folders that are excluded from the documentation

    folders{1} = 'documentation';
    folders{2} = 'manual';
    folders{3} = 'examples';
    folders{4} = 'unitTests';
    folders{5} = fullfile('benchmarks','scenarios');
    folders{6} = fullfile('benchmarks','measurements');
    folders{7} = fullfile('conformance','acado');
    folders{8} = fullfile('conformance','fmincon');
    folders{9} = fullfile('algorithms','convexInterpolationControl','acado');
    folders{10} = fullfile('algorithms','convexInterpolationControl','fmincon');
    folders{11} = fullfile('algorithms','reachsetMPC','acado');
    folders{12} = fullfile('algorithms','reachsetMPC','fmincon');
    folders{13} = fullfile('algorithms','generatorSpaceControl','acado');
    folders{14} = fullfile('algorithms','generatorSpaceControl','fmincon');
    folders{15} = fullfile('algorithms','combinedControl','acado');
    folders{16} = fullfile('algorithms','combinedControl','fmincon');
    folders{17} = fullfile('algorithms','optimizationBasedControl','acado');
    folders{18} = fullfile('algorithms','optimizationBasedControl','fmincon');
    folders{19} = fullfile('algorithms','polynomialControl','acado');
    folders{20} = fullfile('algorithms','polynomialControl','fmincon');
    folders{21} = fullfile('algorithms','safetyNetControl','acado');
    folders{22} = fullfile('algorithms','safetyNetControl','fmincon');

end