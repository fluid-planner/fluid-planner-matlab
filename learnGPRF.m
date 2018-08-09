function [GPRF, fig_handles] = learnGPRF(cf_options, state_window_local, vel_window_local, GPRF)
global GP_METHOD
fig_handles = {};

x_vec = linspace(cf_options.x_lb, cf_options.x_ub, cf_options.num_test);
y_vec = linspace(cf_options.y_lb, cf_options.y_ub, cf_options.num_test);
[X_quiv,Y_quiv] = meshgrid(x_vec, y_vec);

% flatten state and velocity from cell to matrix
state = cell2mat(state_window_local);
vel = cell2mat(vel_window_local);

% create GPRF local grid

if (strcmp(GP_METHOD, 'LIBGP'))
    % method 1: using libgp
    GPRF.libgp_x.clear();
    GPRF.libgp_y.clear();
    GPRF.libgp_x.train(state, vel(1, :)');
    GPRF.libgp_y.train(state, vel(2, :)');
    
    [Vx_PRED, Vx_VAR] = GPRF.libgp_x.predictWithVariance([X_quiv(:), Y_quiv(:)]');
    [Vy_PRED, Vy_VAR] = GPRF.libgp_y.predictWithVariance([X_quiv(:), Y_quiv(:)]');
    Vx_SD = sqrt(Vx_VAR);
    Vy_SD = sqrt(Vy_VAR);
elseif(strcmp(GP_METHOD, 'GPML'))
    % method 2: using gpml
    covfcn = {'covSum', {'covSEiso', 'covNoise'}};
    [Vx_PRED, Vx_SD, ~, ~] = gp(cf_options.hypx, cf_options.inffcn, cf_options.meanfcn, covfcn, cf_options.likfcn, state', vel(1, :)', [X_quiv(:), Y_quiv(:)]);
    [Vy_PRED, Vy_SD, ~, ~] = gp(cf_options.hypy, cf_options.inffcn, cf_options.meanfcn, covfcn, cf_options.likfcn, state', vel(2, :)', [X_quiv(:), Y_quiv(:)]);
end

U = reshape(Vx_PRED, size(X_quiv));
V = reshape(Vy_PRED, size(Y_quiv));

% convert U and V back to velocities

U_SD = reshape(Vx_SD, size(X_quiv));
V_SD = reshape(Vy_SD, size(Y_quiv));

% compute confidence so as to plot the arrows with the appropriate
% colors
conf = sqrt(U_SD + V_SD);
conf = conf/max(max(conf));

GPRF = struct('U_PRED', U , 'U_SD', U_SD, 'V_PRED', V, 'V_SD', V_SD, 'conf', conf, 'X_quiv', X_quiv, 'Y_quiv', Y_quiv, 'state', state, 'vel', vel, ...
              'libgp_x', GPRF.libgp_x, 'libgp_y', GPRF.libgp_y);


%% plot GPRF
% add quiver of training data

for i = 1:10:length(vel)          % plot every nth point
     fig_handles{end+1} = quiver(-state(2, i), state(1, i), -vel(2, i), vel(1, i), 'LineWidth', 2, 'Color', [0.5, 0.5, 0.5], 'MaxHeadSize', 1, 'AutoScale', 'on', 'AutoScaleFactor', 0.1);
end

% add quiver of local map
for i = 1:cf_options.num_test
    for j = 1:cf_options.num_test
        fig_handles{end+1} = quiver(-Y_quiv(i, j), X_quiv(i, j), -V(i, j), U(i, j), 'Color', [1 - conf(i, j), 0.7, 0.7],'MaxHeadSize', 2, 'AutoScale', 'on', 'AutoScaleFactor', 0.2);
    end
end
 
drawnow
   
end
