function buildHpvcAlgorithmCode()
%BUILDHPVCALGORITHMCODE Generate portable code for the HPVC control core.
%
% This builds only the hardware-independent control algorithm. It does not
% create an RPi executable and does not include UDP or TC375 device I/O.

rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(rootDir));
setupRCCarProject;

models = {'RCCarPerceptionAdapter', 'RCCarLcaSupervisor', ...
    'RCCarLateralController', 'RCCarAutonomousSystem'};
for k = 1:numel(models)
    load_system(models{k});
end
cleanup = onCleanup(@() closeModels(models));

target = 'grt.tlc';
if license('test', 'RTW_Embedded_Coder')
    target = 'ert.tlc';
end

for k = 1:numel(models)
    set_param(models{k}, ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'FixedStepDiscrete', ...
        'FixedStep', 'rccar.SampleTime', ...
        'SystemTargetFile', target);
    set_param(getActiveConfigSet(models{k}), 'GenCodeOnly', 'on');
end

fprintf('Generating hardware-independent HPVC control code with %s...\n', target);
slbuild('RCCarAutonomousSystem');
fprintf(['Code generation complete. Device UDP and TC375 I/O are intentionally ' ...
    'outside this model.\n']);
clear cleanup
end

function closeModels(models)
for k = numel(models):-1:1
    if bdIsLoaded(models{k})
        close_system(models{k}, 0);
    end
end
end
