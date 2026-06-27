function [linkValid, packetAgeS, sequenceFresh] = ...
    middleLinkMonitor(currentTimeS, packetValid, sequence, timeoutS, reset)
%MIDDLELINKMONITOR Monitor MIDDLE packet freshness on the HPVC clock.
%#codegen
%
% The sender timestamps use the MIDDLE monotonic clock and must not be
% subtracted from currentTimeS. Freshness is measured from the local time at
% which HPVC observes a valid packet with a new sequence number.

persistent initialized lastSequence lastFreshTimeS

if isempty(initialized) || reset
    initialized = false;
    lastSequence = uint32(0);
    lastFreshTimeS = double(currentTimeS);
end

sequenceFresh = false;
if packetValid
    staleTimeS = max(0.0, double(currentTimeS) - lastFreshTimeS);
    restartResyncS = max(1.0, 5.0 * double(timeoutS));
    senderRestarted = initialized && sequence ~= lastSequence && ...
        staleTimeS >= restartResyncS;
    if ~initialized || isSequenceNewer(sequence, lastSequence) || senderRestarted
        initialized = true;
        lastSequence = sequence;
        lastFreshTimeS = double(currentTimeS);
        sequenceFresh = true;
    end
end

packetAgeS = max(0.0, double(currentTimeS) - lastFreshTimeS);
linkValid = initialized && packetAgeS <= double(timeoutS);
end

function newer = isSequenceNewer(candidate, reference)
% Half-range modular comparison handles uint32 rollover and old packets.
modulus = uint64(4294967296);
difference = mod(uint64(candidate) + modulus - uint64(reference), modulus);
newer = difference > uint64(0) && difference < uint64(2147483648);
end
