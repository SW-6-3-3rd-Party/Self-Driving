function [distancesM, validMask, sequence, timestampMs, tofDiag, packetValid] = ...
    frontAebDecodePacket(data, receiveStatus)
%FRONTAEBDECODEPACKET Decode the TC375 Front AEB1 sensor UDP payload.
%#codegen
%
% distancesM order: [left ultrasonic; right ultrasonic; front ToF].
% validMask bit order: bit0 left, bit1 right, bit2 ToF.

distancesM = single([NaN; NaN; NaN]);
validMask = uint8(0);
sequence = uint32(0);
timestampMs = uint32(0);
tofDiag = uint16(0);
packetValid = false;

bytes = uint8(data(:));
if ~receiveStatus || numel(bytes) < 22
    return
end

magicValid = bytes(1) == uint8(65) && ...  % A
    bytes(2) == uint8(69) && ...            % E
    bytes(3) == uint8(66) && ...            % B
    bytes(4) == uint8(49);                  % 1
versionValid = bytes(5) == uint8(1);
if ~(magicValid && versionValid)
    return
end

validMask = bytes(6);
tofDiag = readU16(bytes, 7);
sequence = readU32(bytes, 9);
timestampMs = readU32(bytes, 13);
tofCmX10 = readU16(bytes, 17);
leftCmX10 = readU16(bytes, 19);
rightCmX10 = readU16(bytes, 21);

distancesM(1) = single(double(leftCmX10) / 1000.0);
distancesM(2) = single(double(rightCmX10) / 1000.0);
distancesM(3) = single(double(tofCmX10) / 1000.0);
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
