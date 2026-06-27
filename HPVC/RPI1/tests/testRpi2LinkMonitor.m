function testRpi2LinkMonitor()
%TESTRPI2LINKMONITOR Unit tests for local receive-time freshness logic.

[valid, age, fresh] = rpi2LinkMonitor(0.0, false, uint32(0), 0.2, true);
assert(~valid && age == 0.0 && ~fresh);

[valid, age, fresh] = rpi2LinkMonitor(1.0, true, uint32(10), 0.2, false);
assert(valid && age == 0.0 && fresh);

[valid, age, fresh] = rpi2LinkMonitor(1.1, true, uint32(10), 0.2, false);
assert(valid && abs(age - 0.1) < 1e-12 && ~fresh);

[valid, age, fresh] = rpi2LinkMonitor(1.21, false, uint32(10), 0.2, false);
assert(~valid && abs(age - 0.21) < 1e-12 && ~fresh);

[valid, ~, fresh] = rpi2LinkMonitor(1.22, true, uint32(9), 0.2, false);
assert(~valid && ~fresh, 'Out-of-order packets must not refresh the link.');

[valid, ~, fresh] = rpi2LinkMonitor(2.01, true, uint32(10), 0.2, false);
assert(~valid && ~fresh, 'Repeated sequences must not resynchronize the link.');

[valid, age, fresh] = rpi2LinkMonitor(2.02, true, uint32(1), 0.2, false);
assert(valid && age == 0.0 && fresh, ...
    'A changed sequence after a long outage must resynchronize a restarted sender.');

rpi2LinkMonitor(3.0, true, uint32(4294967295), 0.2, true);
[valid, age, fresh] = rpi2LinkMonitor(3.05, true, uint32(0), 0.2, false);
assert(valid && age == 0.0 && fresh, 'uint32 sequence rollover must work.');

fprintf('RPi #2 link monitor tests passed.\n');
end
