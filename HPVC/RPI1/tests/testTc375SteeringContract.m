function testTc375SteeringContract
%TESTTC375STEERINGCONTRACT Verify the Front TC375 steering wire contract.

packet = tc375EncodeSteeringPacket(uint32(hex2dec('12345678')), ...
    uint64(123456789), single(0.25), single(1.0), uint8(1), uint8(5), ...
    uint16(hex2dec('ABCD')));

assert(isequal(size(packet), [40 1]));
assert(isequal(packet(1:4)', uint8('R1SC')));
assert(packet(5) == 1 && packet(6) == 1 && packet(7) == 5 && packet(8) == 32);
assert(typecast(packet(9:12)', 'uint32') == uint32(hex2dec('12345678')));
assert(typecast(packet(13:20)', 'uint64') == uint64(123456789));
assert(typecast(packet(21:24)', 'single') == single(0.25));
assert(typecast(packet(25:28)', 'single') == single(1.0));
assert(typecast(packet(29:30)', 'uint16') == uint16(hex2dec('ABCD')));
assert(all(packet(31:36) == 0));

expectedCrc = uint32(hex2dec('694862AB'));
assert(typecast(packet(37:40)', 'uint32') == expectedCrc, ...
    'Canonical packet CRC changed; update all endpoints only with a version change.');
fprintf('Front TC375 steering protocol contract: passed\n');
end
