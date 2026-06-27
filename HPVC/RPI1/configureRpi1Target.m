function configureRpi1Target(deviceAddress, username, password)
%CONFIGURERPI1TARGET Bind RPI1Deployment to the RPi #1 board only.

if nargin < 2 || strlength(string(username)) == 0
    username = 'pi';
end
if nargin < 3
    password = '';
end

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(rootDir));
setupRCCarProject;
rccar = evalin('base', 'rccar');
deviceAddress = char(string(deviceAddress));
username = char(string(username));

assert(~isempty(deviceAddress), 'RPi #1 IP address is required.');
assert(~strcmp(deviceAddress, '0.0.0.0'), 'Enter the actual RPi #1 IP address.');
assert(~strcmp(deviceAddress, rccar.Rpi2Address), ...
    'RPi #1 address must not equal the configured RPi #2 address (%s).', ...
    rccar.Rpi2Address);

model = 'RPI1Deployment';
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
fprintf('RPI1Deployment target set to %s (user %s).\n', deviceAddress, username);
end
