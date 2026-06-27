function testRpi2Contract()
%TESTRPI2CONTRACT Verify the Python/MATLAB v1 protocol contract.

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
pythonExe = resolvePython(rootDir);
fixtureFile = [tempname '.bin'];
cleanup = onCleanup(@() deleteIfPresent(fixtureFile));

command = sprintf('"%s" -m RPI2.generate_contract_fixture "%s"', ...
    pythonExe, fixtureFile);
[status, output] = system(command);
assert(status == 0, 'Python fixture generation failed: %s', output);

data = readBytes(fixtureFile);
assert(numel(data) == 80, 'Protocol v1 packet must be exactly 80 bytes.');
[lane, side, flags, sequence, frameTs, ultraTs, valid] = ...
    rpi2DecodePacket(data, true);

assert(valid);
assert(flags == uint8(15));
assert(sequence == uint32(4294967294));
assert(frameTs == uint64(1234567890123));
assert(ultraTs == uint64(1234567890456));
assert(max(abs(double(lane) - [0.2;0.01;0.1;0.18;0.9; ...
    -0.3;-0.02;-0.12;-0.22;0.8])) < 1e-6);
assert(max(abs(double(side) - [0.75;0.8])) < 1e-6);

corrupt = data;
corrupt(30) = bitxor(corrupt(30), uint8(1));
[~, ~, ~, ~, ~, ~, corruptValid] = rpi2DecodePacket(corrupt, true);
assert(~corruptValid, 'CRC corruption must be rejected.');

fprintf('RPi #2 <-> RPi #1 protocol v1 contract passed.\n');
clear cleanup
end

function pythonExe = resolvePython(rootDir)
venvPython = fullfile(rootDir, '.venv', 'bin', 'python3');
if isfile(venvPython)
    pythonExe = venvPython;
else
    pythonExe = 'python3';
end
end

function data = readBytes(file)
fid = fopen(file, 'rb');
assert(fid >= 0, 'Could not open fixture: %s', file);
cleanup = onCleanup(@() fclose(fid));
data = fread(fid, Inf, '*uint8');
clear cleanup
end

function deleteIfPresent(file)
if isfile(file)
    delete(file);
end
end
