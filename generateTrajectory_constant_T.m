% Copyright 2018 Toyota Research Institute.  All rights reserved.
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

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
