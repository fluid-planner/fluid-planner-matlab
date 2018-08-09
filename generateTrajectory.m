function pos = generateTrajectory(v, omega_vec, duration_vec, dt, pos_init)

pos = pos_init;
m = length(omega_vec);
assert(m == length(duration_vec));

% generate the trajectory
t = 0;
x_prev = pos_init;
for i=1:m
    omega = omega_vec(i);
    while t <= duration_vec(i)
        pos(:, end+1) = dubins(v, omega, t, x_prev);
        t = t + dt;
    end
    t = 0;
    x_prev = pos(:, end);
end

end