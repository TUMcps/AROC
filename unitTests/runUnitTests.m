%% This script runs all unit tests from the AROC toolbox

% get current MATALB path
path = pwd;

% switch to the unit test folder
[pathTest,~,~] = fileparts(mfilename('fullpath'));
cd(pathTest);

% create test suite from all test functions in the folder
suite = testsuite;

% run the test suite
run(suite)

% restore original path
cd(path)