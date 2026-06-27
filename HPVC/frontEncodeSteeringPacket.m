function packet = frontEncodeSteeringPacket(sequence, timestampUs, ...
    steeringAngleRad, maxSteeringRateRadS, controlMode, flags, aliveCount)
%FRONTENCODESTEERINGPACKET Encode the fixed 40-byte steering command packet.
%#codegen

packet = zeros(40, 1, 'uint8');
packet(1:4) = uint8([72; 80; 83; 67]); % HPSC
packet(5) = uint8(1);
packet(6) = uint8(controlMode);
packet(7) = uint8(flags);
packet(8) = uint8(32);
packet(9:12) = littleEndianBytes(uint32(sequence));
packet(13:20) = littleEndianBytes(uint64(timestampUs));
packet(21:24) = typecast(single(steeringAngleRad), 'uint8')';
packet(25:28) = typecast(single(maxSteeringRateRadS), 'uint8')';
packet(29:30) = littleEndianBytes(uint16(aliveCount));
packet(31:36) = uint8(0);
crc = crc32Ieee(packet(1:36));
packet(37:40) = littleEndianBytes(crc);
end

function bytes = littleEndianBytes(value)
bytes = typecast(value, 'uint8')';
end

function crc = crc32Ieee(data)
crc = uint32(hex2dec('FFFFFFFF'));
poly = uint32(hex2dec('EDB88320'));
for k = 1:numel(data)
    crc = bitxor(crc, uint32(data(k)));
    for bit = 1:8
        if bitand(crc, uint32(1)) ~= 0
            crc = bitxor(bitshift(crc, -1), poly);
        else
            crc = bitshift(crc, -1);
        end
    end
end
crc = bitcmp(crc);
end
