% PLOTRCCARRESULTS Plot the steering output saved by RCCarSystemTestBench.

if ~exist('rccarSteering', 'var')
    error('Run TestBench/RCCarSystemTestBench.slx first.');
end

figure('Name', 'RC Car System Results');
plot(rccarSteering.Time, rccarSteering.Data, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Steering angle (rad)');
title('Limited steering command');

