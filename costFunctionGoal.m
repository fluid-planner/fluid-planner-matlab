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

function J = costFunctionGoal(traj, wpts, goal, lambda)
% traj = parameters of the trajectory
% wpts = traj sampled wrt to time

% the objective J we want to optimize

N = length(wpts);
J_unnormalized = [];

omega_des = 1;

for n = 1:N
    J_unnormalized(n, 1) = C_dist(wpts{n}, goal, lambda);
    J_unnormalized(n, 2) = C_smoothness(traj(n, :));
    J_unnormalized(n, 3) = C_curvature(traj(n, :), omega_des);
    J_unnormalized(n, 4) = norm(wpts{n}(:, end) - goal);
end

% J_normalized = J_unnormalized.*[0.01, 1, 1, 1];
wgt = [1, 0, 0, 0];
J_normalized = J_unnormalized.*repmat(wgt, N, 1);       % 2014b
% J_normalized = J_unnormalized.*[1, 0, 0, 0];          % 2018b
J = sum(J_normalized, 2);

% J = J_unnormalized;

end

% distance based cost function
function c = C_dist(wpts, goal, lambda)
c = 0;
for i = 1:length(wpts)
    pt = wpts(:, i);
    c = c + lambda*norm(pt - goal);
end

end

function c = C_curvature(traj, omega_des)
c = 0;
for m = 1:length(traj)/2
    omega = traj(m);
    c = c + norm(omega - omega_des);
end
end

function c = C_smoothness(traj)
c = 0;
for i = 2:length(traj)/2
    c = c + norm(traj(i+1)-traj(i));
end
end
