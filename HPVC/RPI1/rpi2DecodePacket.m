function [laneFeatures, sideDistances, flags, sequence, ...
    frameTimestampUs, ultrasonicTimestampUs, packetValid] = ...
    rpi2DecodePacket(data, receiveStatus)
%RPI2DECODEPACKET Decode and validate the fixed 80-byte RPi #2 UDP packet.
%#codegen

laneFeatures = zeros(10, 1, 'single');
sideDistances = single([NaN; NaN]);
flags = uint8(0);
sequence = uint32(0);
frameTimestampUs = uint64(0);
ultrasonicTimestampUs = uint64(0);
packetValid = false;

bytes = uint8(data(:));
if ~receiveStatus || numel(bytes) ~= 80
    return
end

magicValid = bytes(1) == uint8(82) && ...  % R
    bytes(2) == uint8(80) && ...            % P
    bytes(3) == uint8(50) && ...            % 2
    bytes(4) == uint8(76);                  % L
versionValid = bytes(5) == uint8(1);
payloadCountValid = readU16(bytes, 7) == uint16(12);
receivedCrc = readU32(bytes, 77);
calculatedCrc = calculateCrc32(bytes(1:76));
if ~(magicValid && versionValid && payloadCountValid && ...
        receivedCrc == calculatedCrc)
    return
end

flags = bytes(6);
sequence = readU32(bytes, 9);
frameTimestampUs = readU64(bytes, 13);
ultrasonicTimestampUs = readU64(bytes, 21);

payload = zeros(12, 1, 'single');
for k = 1:12
    payload(k) = readSingle(bytes, 29 + (k - 1) * 4);
end
laneFeatures(:) = payload(1:10);
sideDistances(:) = payload(11:12);
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
