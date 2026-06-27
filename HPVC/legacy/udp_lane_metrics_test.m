% UDP lane metrics receive test for MATLAB.
%
% Python sends four little-endian single values:
%   [lane_detected, offset_m, curvature_m, camera_status]
%
% Run this before Simulink to confirm that MATLAB is receiving packets.

local_port = 5005;
u = udpport("datagram", "IPV4", "LocalPort", local_port);

fprintf("Waiting for UDP lane metrics on port %d...\n", local_port);

while true
    if u.NumDatagramsAvailable > 0
        datagram = read(u, 1, "uint8");
        raw = uint8(datagram.Data);

        if numel(raw) ~= 16
            fprintf("Skipped packet with %d bytes. Expected 16 bytes.\n", numel(raw));
            continue;
        end

        values = typecast(raw, "single");

        lane_detected = values(1);
        offset_m = values(2);
        curvature_m = values(3);
        camera_status = values(4);

        fprintf( ...
            "lane=%.1f, offset=%+.3f m, curvature=%.1f m, camera=%.1f\n", ...
            lane_detected, offset_m, curvature_m, camera_status ...
        );
    end

    pause(0.01);
end
