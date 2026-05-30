function folders = excludedFolders()
% Folders that are excluded from the documentation

    folders{1} = 'documentation';
    folders{2} = 'manual';
    folders{3} = 'examples';
    folders{4} = 'unitTests';
    folders{5} = fullfile('benchmarks','scenarios');
    folders{6} = fullfile('benchmarks','measurements');
    folders{7} = fullfile('conformance','fmincon');
    folders{8} = fullfile('algorithms','convexInterpolationControl','fmincon');
    folders{9} = fullfile('algorithms','reachsetMPC','fmincon');
    folders{10} = fullfile('algorithms','generatorSpaceControl','fmincon');
    folders{11} = fullfile('algorithms','combinedControl','fmincon');
    folders{12} = fullfile('algorithms','optimizationBasedControl','fmincon');
    folders{13} = fullfile('algorithms','polynomialControl','fmincon');
    folders{14} = fullfile('algorithms','safetyNetControl','fmincon');

end