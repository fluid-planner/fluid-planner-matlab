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

