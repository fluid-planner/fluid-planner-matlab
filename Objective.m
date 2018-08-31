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

function J = Objective(traj, wpts, goal, lambda)
% traj = parameters of the trajectory
% wpts = traj sampled wrt to time

% the objective J we want to optimize

J = 0;
for i = 1:length(wpts)
    J = J + C_dist(wpts(:, i), goal, lambda) ;
end

omega_des = 1;
C1 = 0;
for m = 1:length(traj)/2
    C1 = C1 + C_curvature(traj(m), omega_des);
end

J = J+C1;

C2 = C_smoothness(traj);
J = J + C2;

% only evaluate the end point
% J = C(wpts(:, end), goal,lambda); 

end

% distance based cost function
function c = C_dist(pt, goal, lambda)
    c = 1 + lambda*norm(pt - goal);
end

function c = C_curvature(omega, omega_des)
    c = norm(omega - omega_des);
end

function c = C_smoothness(traj)
c = 0;
for i = 2:length(traj)/2
    c = c + norm(traj(i+1)-traj(i));
end
end
