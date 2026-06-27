function buildRpi1DeploymentModel(force)
%BUILDRPI1DEPLOYMENTMODEL Create the standalone RPi #1 UDP receiver model.
%
% Requires Raspberry Pi Blockset. Run the desktop validation model first.

if nargin < 1, force = false; end
rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(rootDir));
setupRCCarProject;
rccar = evalin('base', 'rccar');

raspiUdpSource = findRaspberryPiUdpReceive();
raspiUdpSink = findRaspberryPiUdpSend();
desktopModel = 'RPI1DesktopValidation';
deploymentModel = 'RPI1Deployment';
desktopFile = fullfile(rootDir, 'RPI1', 'Models', [desktopModel '.slx']);
deploymentFile = fullfile(rootDir, 'RPI1', 'Models', [deploymentModel '.slx']);

if isfile(deploymentFile) && ~force
    fprintf('Keeping existing model: %s\n', deploymentFile);
    return
end
buildRpi1DesktopValidationModel(true);
load_system(desktopFile);
if bdIsLoaded(deploymentModel), close_system(deploymentModel, 0); end
save_system(desktopModel, deploymentFile);
close_system(desktopModel, 0);
load_system(deploymentModel);

oldReceive = [deploymentModel '/UDP Receive from RPi2'];
front = [deploymentModel '/Decode and Monitor RPi2'];
deleteIncomingLines(front, 1:2);
delete_block(oldReceive);

receive = [deploymentModel '/UDP Receive from RPi2'];
add_block(raspiUdpSource, receive, 'Position', [35 75 205 145]);
configureUdpReceiveMask(receive, rccar);
add_block('simulink/Logic and Bit Operations/Compare To Constant', ...
    [deploymentModel '/Received 80 Bytes'], 'const', '80', 'relop', '==', ...
    'Position', [225 150 350 180]);

receivePorts = get_param(receive, 'PortHandles');
frontPorts = get_param(front, 'PortHandles');
sizeCheckPorts = get_param([deploymentModel '/Received 80 Bytes'], 'PortHandles');
add_line(deploymentModel, receivePorts.Outport(1), frontPorts.Inport(1), ...
    'autorouting', 'on');
add_line(deploymentModel, receivePorts.Outport(2), sizeCheckPorts.Inport(1), ...
    'autorouting', 'on');
add_line(deploymentModel, sizeCheckPorts.Outport(1), frontPorts.Inport(2), ...
    'autorouting', 'on');

% Never enable steering automatically on boot. Replace this with the
% vehicle supervisor enable signal after stationary safety tests.
set_param([deploymentModel '/LKAS Enable'], 'Value', 'false');
replaceLogWithTerminator(deploymentModel, 'Steering Command');
replaceLogWithTerminator(deploymentModel, 'Link Valid');
replaceLogWithTerminator(deploymentModel, 'Packet Age s');
addTc375CommandUdp(deploymentModel, raspiUdpSink, frontPorts, rccar);
addDiagnosticUdp(deploymentModel, raspiUdpSink, frontPorts, rccar);

set_param(deploymentModel, 'StopTime', 'inf', 'EnablePacing', 'off');
set_param(deploymentModel, 'HardwareBoard', 'Raspberry Pi (64bit)');
clearInheritedTargetAddress(deploymentModel);
note = Simulink.Annotation(deploymentModel, [ ...
    'RPi #1 standalone receiver. LKAS Enable defaults to false. ' ...
    'TC375 UDP sends EmergencyCenter only until LKAS is explicitly enabled.']);
note.Position = [35 690 1060 725];
save_system(deploymentModel, deploymentFile);
close_system(deploymentModel, 0);
fprintf('Created %s\n', deploymentFile);
end

function clearInheritedTargetAddress(model)
% Do not inherit MATLAB's last-used board, which may be RPi #2.
config = getActiveConfigSet(model);
data = codertarget.data.getData(config);
data.BoardParameters.DeviceAddress = '0.0.0.0';
codertarget.data.setData(config, data);
end

function source = findRaspberryPiUdpReceive()
libraries = {'raspberrypiNetworklib', 'raspberrypilib'};
for k = 1:numel(libraries)
    try
        load_system(libraries{k});
        blocks = find_system(libraries{k}, 'LookUnderMasks', 'all', ...
            'FollowLinks', 'on', 'Name', 'UDP Receive');
        if ~isempty(blocks)
            source = blocks{1};
            return
        end
    catch
    end
end
error('rccar:RaspberryPiBlocksetMissing', [ ...
    'Raspberry Pi Blockset with the UDP Receive block is not installed. ' ...
    'Install it, restart MATLAB, and rerun buildRpi1DeploymentModel(true).']);
end

function source = findRaspberryPiUdpSend()
libraries = {'raspberrypiNetworklib', 'raspberrypilib'};
for k = 1:numel(libraries)
    try
        load_system(libraries{k});
        blocks = find_system(libraries{k}, 'LookUnderMasks', 'all', ...
            'FollowLinks', 'on', 'Name', 'UDP Send');
        if ~isempty(blocks)
            source = blocks{1};
            return
        end
    catch
    end
end
error('rccar:RaspberryPiBlocksetMissing', ...
    'Raspberry Pi Blockset with the UDP Send block is not installed.');
end

function configureUdpReceiveMask(block, rccar)
setMaskValue(block, 'local.*port', num2str(rccar.Rpi2ListenPort));
setMaskValue(block, 'data type', 'uint8');
setMaskValue(block, 'data size', '80');
setMaskValue(block, 'sample time', 'rccar.SampleTime');
end

function setMaskValue(block, promptPattern, value)
mask = Simulink.Mask.get(block);
assert(~isempty(mask), 'Expected a masked Raspberry Pi UDP Receive block.');
parameters = mask.Parameters;
for k = 1:numel(parameters)
    if ~isempty(regexpi(parameters(k).Prompt, promptPattern, 'once'))
        set_param(block, parameters(k).Name, value);
        return
    end
end
error('Could not find UDP Receive mask parameter matching "%s".', promptPattern);
end

function deleteIncomingLines(block, portIndices)
ports = get_param(block, 'PortHandles');
for index = portIndices
    line = get_param(ports.Inport(index), 'Line');
    if line ~= -1, delete_line(line); end
end
end

function replaceLogWithTerminator(model, name)
path = [model '/' name];
ports = get_param(path, 'PortHandles');
line = get_param(ports.Inport(1), 'Line');
sourcePort = get_param(line, 'SrcPortHandle');
position = get_param(path, 'Position');
delete_line(line);
delete_block(path);
add_block('simulink/Sinks/Terminator', path, 'Position', position);
ports = get_param(path, 'PortHandles');
add_line(model, sourcePort, ports.Inport(1), 'autorouting', 'on');
end

function addTc375CommandUdp(model, udpSendSource, frontPorts, rccar)
encoder = [model '/Encode TC375 Steering Command'];
add_block('simulink/User-Defined Functions/MATLAB Function', encoder, ...
    'Position', [1010 475 1215 625]);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', encoder);
chart.Script = tc375EncoderScript();
setChartData(chart, 'steeringCommand', 'single', '1');
setChartData(chart, 'controlValid', 'boolean', '1');
setChartData(chart, 'lkasEnable', 'boolean', '1');
setChartData(chart, 'localTimeS', 'double', '1');
setChartData(chart, 'maxSteerRad', 'single', '1');
setChartData(chart, 'maxSteerRateRadS', 'single', '1');
setChartData(chart, 'packet', 'uint8', '[40 1]');

maxSteer = [model '/TC375 Max Steering rad'];
add_block('simulink/Sources/Constant', maxSteer, ...
    'Value', 'single(rccar.MaxSteerRad)', 'OutDataTypeStr', 'single', ...
    'Position', [815 555 945 575]);
maxRate = [model '/TC375 Max Steering Rate'];
add_block('simulink/Sources/Constant', maxRate, ...
    'Value', 'single(rccar.MaxSteerRate)', 'OutDataTypeStr', 'single', ...
    'Position', [815 605 945 625]);

steeringTerminator = [model '/Steering Command'];
steeringLine = get_param(get_param(steeringTerminator, 'PortHandles').Inport(1), 'Line');
steeringSource = get_param(steeringLine, 'SrcPortHandle');
encoderPorts = get_param(encoder, 'PortHandles');
sources = [steeringSource, frontPorts.Outport(3), ...
    get_param([model '/LKAS Enable'], 'PortHandles').Outport(1), ...
    get_param([model '/RPi1 Local Time'], 'PortHandles').Outport(1), ...
    get_param(maxSteer, 'PortHandles').Outport(1), ...
    get_param(maxRate, 'PortHandles').Outport(1)];
for k = 1:numel(sources)
    add_line(model, sources(k), encoderPorts.Inport(k), 'autorouting', 'on');
end

udpSend = [model '/UDP Send to TC375 Front'];
add_block(udpSendSource, udpSend, 'Position', [1265 515 1445 575]);
set_param(udpSend, 'remoteURL', ['''' rccar.Tc375FrontAddress ''''], ...
    'remotePort', num2str(rccar.Tc375FrontCommandPort), ...
    'localURL', '''0.0.0.0''', 'localPortSource', 'Specify via dialog', ...
    'localPort', num2str(rccar.Tc375FrontSourcePort), ...
    'sampletime', 'rccar.SampleTime', 'separateLengthPort', '0');
add_line(model, encoderPorts.Outport(1), ...
    get_param(udpSend, 'PortHandles').Inport(1), 'autorouting', 'on');
end

function addDiagnosticUdp(model, udpSendSource, frontPorts, rccar)
encoder = [model '/Encode RPi1 Diagnostic'];
add_block('simulink/User-Defined Functions/MATLAB Function', encoder, ...
    'Position', [1010 300 1215 440]);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', encoder);
chart.Script = diagnosticEncoderScript();
setChartData(chart, 'packetValid', 'boolean', '1');
setChartData(chart, 'linkValid', 'boolean', '1');
setChartData(chart, 'controlValid', 'boolean', '1');
setChartData(chart, 'rpi2Flags', 'uint8', '1');
setChartData(chart, 'sequence', 'uint32', '1');
setChartData(chart, 'packetAgeS', 'double', '1');
setChartData(chart, 'steeringCommand', 'single', '1');
setChartData(chart, 'localTimeS', 'double', '1');
setChartData(chart, 'packet', 'uint8', '[40 1]');

steeringCast = [model '/Diagnostic Steering single'];
add_block('simulink/Signal Attributes/Data Type Conversion', steeringCast, ...
    'OutDataTypeStr', 'single', 'Position', [910 405 975 435]);
steeringTerminator = [model '/Steering Command'];
steeringLine = get_param(get_param(steeringTerminator, 'PortHandles').Inport(1), 'Line');
steeringSource = get_param(steeringLine, 'SrcPortHandle');
add_line(model, steeringSource, get_param(steeringCast, 'PortHandles').Inport(1), ...
    'autorouting', 'on');

encoderPorts = get_param(encoder, 'PortHandles');
diagnosticSources = [frontPorts.Outport(7), frontPorts.Outport(8), ...
    frontPorts.Outport(3), frontPorts.Outport(6), frontPorts.Outport(4), ...
    frontPorts.Outport(5), get_param(steeringCast, 'PortHandles').Outport(1), ...
    get_param([model '/RPi1 Local Time'], 'PortHandles').Outport(1)];
for k = 1:numel(diagnosticSources)
    add_line(model, diagnosticSources(k), encoderPorts.Inport(k), 'autorouting', 'on');
end

udpSend = [model '/UDP Send Diagnostic to Mac'];
add_block(udpSendSource, udpSend, 'Position', [1265 340 1445 400]);
set_param(udpSend, 'remoteURL', ['''' rccar.Rpi1DiagnosticHost ''''], ...
    'remotePort', num2str(rccar.Rpi1DiagnosticPort), ...
    'localURL', '''0.0.0.0''', 'localPortSource', 'Automatically determine', ...
    'sampletime', 'rccar.SampleTime', 'separateLengthPort', '0');
add_line(model, encoderPorts.Outport(1), ...
    get_param(udpSend, 'PortHandles').Inport(1), 'autorouting', 'on');
end

function setChartData(chart, name, dataType, sizeExpression)
item = find(chart, '-isa', 'Stateflow.Data', 'Name', name);
assert(isscalar(item), 'Could not find MATLAB Function data item: %s.', name);
item.DataType = dataType;
item.Props.Array.Size = sizeExpression;
end

function script = diagnosticEncoderScript()
script = sprintf([ ...
    'function packet = encodeDiagnostic(packetValid, linkValid, controlValid, rpi2Flags, sequence, packetAgeS, steeringCommand, localTimeS)\n' ...
    '%%#codegen\n' ...
    'packet=zeros(40,1,''uint8'');\n' ...
    'packet(1:4)=uint8([82;49;68;71]); packet(5)=uint8(1);\n' ...
    'status=uint8(packetValid)+bitshift(uint8(linkValid),1)+bitshift(uint8(controlValid),2);\n' ...
    'packet(6)=status; packet(7)=rpi2Flags; packet(8)=uint8(0);\n' ...
    'for k=0:3, packet(9+k)=uint8(bitand(bitshift(sequence,-8*k),uint32(255))); end\n' ...
    'ageBytes=typecast(double(packetAgeS),''uint8''); packet(13:20)=ageBytes(:);\n' ...
    'steerBytes=typecast(single(steeringCommand),''uint8''); packet(21:24)=steerBytes(:);\n' ...
    'timeBytes=typecast(double(localTimeS),''uint8''); packet(25:32)=timeBytes(:);\n' ...
    'packet(33:36)=uint8(0); crc=uint32(hex2dec(''FFFFFFFF''));\n' ...
    'poly=uint32(hex2dec(''EDB88320''));\n' ...
    'for k=1:36\n' ...
    ' crc=bitxor(crc,uint32(packet(k)));\n' ...
    ' for bit=1:8\n' ...
    '  if bitand(crc,uint32(1))~=0, crc=bitxor(bitshift(crc,-1),poly); else, crc=bitshift(crc,-1); end\n' ...
    ' end\n' ...
    'end\n' ...
    'crc=bitcmp(crc); for k=0:3, packet(37+k)=uint8(bitand(bitshift(crc,-8*k),uint32(255))); end\n' ...
    'end\n']);
end

function script = tc375EncoderScript()
script = sprintf([ ...
    'function packet = encodeTc375(steeringCommand, controlValid, lkasEnable, localTimeS, maxSteerRad, maxSteerRateRadS)\n' ...
    '%%#codegen\n' ...
    'persistent sequence aliveCount\n' ...
    'if isempty(sequence), sequence=uint32(0); aliveCount=uint16(0); end\n' ...
    'steeringValid=controlValid && lkasEnable && isfinite(steeringCommand) && abs(steeringCommand)<=maxSteerRad;\n' ...
    'if steeringValid, safeAngle=steeringCommand; else, safeAngle=single(0); end\n' ...
    'emergencyCenter=~steeringValid; controlMode=uint8(steeringValid);\n' ...
    'flags=uint8(steeringValid)+bitshift(uint8(emergencyCenter),1)+bitshift(uint8(controlValid),2);\n' ...
    'timestampUs=uint64(max(localTimeS,0.0)*1.0e6);\n' ...
    'packet=tc375EncodeSteeringPacket(sequence,timestampUs,safeAngle,maxSteerRateRadS,controlMode,flags,aliveCount);\n' ...
    'sequence=sequence+uint32(1); aliveCount=aliveCount+uint16(1);\n' ...
    'end\n']);
end
