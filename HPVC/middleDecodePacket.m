function [laneFeatures, sideDistances, flags, sequence, ...
    frameTimestampUs, ultrasonicTimestampUs, packetValid, ...
    personDetections, personCount] = middleDecodePacket(data, receiveStatus)
%MIDDLEDECODEPACKET Decode and validate MIDDLE UDP perception packets.
%#codegen
%
% Supports the current protocol v2 packet with YOLO person detections and the
% legacy v1 80-byte lane/ultrasonic packet. The first seven outputs are kept
% backward-compatible with the original HPVC Simulink frontend.

laneFeatures = zeros(10, 1, 'single');
sideDistances = single([NaN; NaN]);
flags = uint8(0);
sequence = uint32(0);
frameTimestampUs = uint64(0);
ultrasonicTimestampUs = uint64(0);
packetValid = false;
personDetections = zeros(6, 3, 'single');
personCount = uint8(0);

bytes = uint8(data(:));
packetLength = numel(bytes);
if ~receiveStatus || ~(packetLength == 80 || packetLength == 156)
    return
end

magicValid = bytes(1) == uint8(77) && ...  % M
    bytes(2) == uint8(73) && ...            % I
    bytes(3) == uint8(68) && ...            % D
    bytes(4) == uint8(50);                  % 2

version = bytes(5);
if version == uint8(1)
    expectedLength = 80;
    expectedFloatCount = uint16(12);
elseif version == uint8(2)
    expectedLength = 156;
    expectedFloatCount = uint16(31);
else
    return
end

if packetLength ~= expectedLength
    return
end

payloadCountValid = readU16(bytes, 7) == expectedFloatCount;
crcStart = packetLength - 3;
receivedCrc = readU32(bytes, crcStart);
calculatedCrc = calculateCrc32(bytes(1:packetLength - 4));
if ~(magicValid && payloadCountValid && receivedCrc == calculatedCrc)
    return
end

flags = bytes(6);
sequence = readU32(bytes, 9);
frameTimestampUs = readU64(bytes, 13);
ultrasonicTimestampUs = readU64(bytes, 21);

payload = zeros(31, 1, 'single');
for k = 1:double(expectedFloatCount)
    payload(k) = readSingle(bytes, 29 + (k - 1) * 4);
end
laneFeatures(:) = payload(1:10);
sideDistances(:) = payload(11:12);

if version == uint8(2)
    rawCount = floor(double(payload(13)));
    rawCount = min(max(rawCount, 0), 3);
    personCount = uint8(rawCount);
    for index = 1:3
        payloadIndex = 14 + (index - 1) * 6;
        personDetections(:, index) = payload(payloadIndex:payloadIndex + 5);
    end
end

packetValid = true;
end

function value = readU16(bytes, startIndex)
value = bitor(uint16(bytes(startIndex)), ...
    bitshift(uint16(bytes(startIndex + 1)), 8));
end

function value = readU32(bytes, startIndex)
value = uint32(bytes(startIndex));
for offset = 1:3
    value = bitor(value, bitshift(uint32(bytes(startIndex + offset)), 8 * offset));
end
end

function value = readU64(bytes, startIndex)
low = uint64(readU32(bytes, startIndex));
high = uint64(readU32(bytes, startIndex + 4));
value = bitor(low, bitshift(high, 32));
end

function value = readSingle(bytes, startIndex)
bits = readU32(bytes, startIndex);
value = typecast(bits, 'single');
end

function crc = calculateCrc32(bytes)
crc = uint32(hex2dec('FFFFFFFF'));
polynomial = uint32(hex2dec('EDB88320'));
for k = 1:numel(bytes)
    crc = bitxor(crc, uint32(bytes(k)));
    for bit = 1:8
        if bitand(crc, uint32(1)) ~= 0
            crc = bitxor(bitshift(crc, -1), polynomial);
        else
            crc = bitshift(crc, -1);
        end
    end
end
crc = bitcmp(crc);
end
