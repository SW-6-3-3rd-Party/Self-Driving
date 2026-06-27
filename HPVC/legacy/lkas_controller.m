function [steer_cmd, lkas_active, lane_valid] = lkas_controller(lane_detected, offset_m, curvature_m, camera_status, vehicle_speed)
%LKAS_CONTROLLER Lane keeping assist controller for Simulink.
%
% Inputs from UDP lane detector:
%   lane_detected  - 1.0 when both lane boundaries are detected
%   offset_m       - vehicle center offset from lane center [m]
%   curvature_m    - lane curvature radius [m], 9999 means nearly straight
%   camera_status  - 1.0 when camera stream is valid
%   vehicle_speed  - vehicle speed [m/s]
%
% Outputs:
%   steer_cmd   - steering command [rad]
%   lkas_active - true when LKAS is actively controlling
%   lane_valid  - true when received lane metrics are usable

K_offset = 0.45;
K_curve = 0.25;
max_steer = 0.45;
min_speed = 1.0;

lane_valid = (lane_detected > 0.5) && ...
             (camera_status > 0.5) && ...
             (curvature_m > 1.0);

lkas_active = lane_valid && (vehicle_speed >= min_speed);

if ~lkas_active
    steer_cmd = 0.0;
    return;
end

% Python detector convention:
% offset_m > 0 means vehicle is left of lane center, so steer right.
steer_offset = -K_offset * offset_m;

if curvature_m >= 9999.0
    steer_curve = 0.0;
else
    steer_curve = K_curve * vehicle_speed / curvature_m;
end

steer_cmd = steer_offset + steer_curve;

if steer_cmd > max_steer
    steer_cmd = max_steer;
elseif steer_cmd < -max_steer
    steer_cmd = -max_steer;
end
end
