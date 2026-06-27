function testMiddleContract()
%TESTMIDDLECONTRACT Verify the Python/MATLAB v2 protocol contract.

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
pythonExe = resolvePython(rootDir);
fixtureFile = [tempname '.bin'];
cleanup = onCleanup(@() deleteIfPresent(fixtureFile));

command = sprintf('"%s" -m MIDDLE.generate_contract_fixture "%s"', ...
    pythonExe, fixtureFile);
[status, output] = system(command);
assert(status == 0, 'Python fixture generation failed: %s', output);

data = readBytes(fixtureFile);
assert(numel(data) == 156, 'Protocol v2 packet must be exactly 156 bytes.');
[lane, side, flags, sequence, frameTs, ultraTs, valid, persons, personCount] = ...
    middleDecodePacket(data, true);

assert(valid);
assert(flags == uint8(31));
assert(sequence == uint32(4294967294));
assert(frameTs == uint64(1234567890123));
assert(ultraTs == uint64(1234567890456));
assert(max(abs(double(lane) - [0.2;0.01;0.1;0.18;0.9; ...
    -0.3;-0.02;-0.12;-0.22;0.8])) < 1e-6);
assert(max(abs(double(side) - [0.75;0.8])) < 1e-6);
assert(personCount == uint8(1));
assert(abs(double(persons(2, 1)) - 0.86) < 1e-6);
assert(abs(double(persons(3, 1)) - 0.52) < 1e-6);

corrupt = data;
corrupt(30) = bitxor(corrupt(30), uint8(1));
[~, ~, ~, ~, ~, ~, corruptValid] = middleDecodePacket(corrupt, true);
assert(~corruptValid, 'CRC corruption must be rejected.');

fprintf('MIDDLE <-> HPVC protocol v2 contract passed.\n');
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
