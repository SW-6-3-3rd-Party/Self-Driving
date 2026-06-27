function configureHpvcTarget(deviceAddress, username, password)
%CONFIGUREHPVCTARGET Bind HPVCDeployment to the HPVC board only.

if nargin < 2 || strlength(string(username)) == 0
    username = 'pi';
end
if nargin < 3
    password = '';
end

rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(rootDir));
setupRCCarProject;
rccar = evalin('base', 'rccar');
deviceAddress = char(string(deviceAddress));
username = char(string(username));

assert(~isempty(deviceAddress), 'HPVC IP address is required.');
assert(~strcmp(deviceAddress, '0.0.0.0'), 'Enter the actual HPVC IP address.');
assert(~strcmp(deviceAddress, rccar.MiddleAddress), ...
    'HPVC address must not equal the configured MIDDLE address (%s).', ...
    rccar.MiddleAddress);

model = 'HPVCDeployment';
load_system(model);
config = getActiveConfigSet(model);
data = codertarget.data.getData(config);
data.BoardParameters.DeviceAddress = deviceAddress;
codertarget.data.setData(config, data);
save_system(model);

target = 'Raspberry Pi (64bit)';
rpiutils.linux.remotebuild.setDeviceAddress([], [], deviceAddress, target);
rpiutils.linux.remotebuild.setUsername([], [], username, target);
if ~isempty(password)
    rpiutils.linux.remotebuild.setPassword([], [], char(string(password)), target);
end
close_system(model, 0);
fprintf('HPVCDeployment target set to %s (user %s).\n', deviceAddress, username);
end
