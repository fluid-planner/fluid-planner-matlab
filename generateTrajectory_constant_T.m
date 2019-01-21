function [wpts, vels, start_pts] = generateTrajectory_constant_T(vel_vec, omega_vec, T, dt, start)

rotz = @(alpha) [cos(alpha) -sin(alpha) 0; ...
                sin(alpha)  cos(alpha) 0; ...
                    0           0       1];
                
m = length(omega_vec);
assert(m == length(vel_vec));

% generate the trajectory

x_prev = start;
wpts = start;
vels = [vel_vec(1); 0; 0];
start_pts = start;
for i=1:m
    omega = omega_vec(i);
    v = vel_vec(i);
    t = dt;
    while t <= T
        wpts(:, end+1) = dubins(v, omega, t, x_prev);
        vels(:, end+1) = rotz(start(3))'*rotz(wpts(3, end))*[v; 0; 0];
        t = t + dt;
    end
    x_prev = wpts(:, end);
    start_pts(:, end+1) = x_prev;
end
start_pts(:, end) = [];
end