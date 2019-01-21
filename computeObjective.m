%% NOT USED ATM

function J = computeObjective(cf_type, cf_options, traj, wpts)

if strcmp(cf_type, 'GOAL')
    J = costFunctionGoal(traj, wpts, goal, lambda);
elseif strcmp(cf_Type, 'GPRF')
    J = costFunctionGPRF(cf_options, traj, wpts);
end

end