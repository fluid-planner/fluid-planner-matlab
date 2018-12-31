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

function J = costFunctionGPRF(cf_options, dt, Z, Paths, Vels, GPRF, start)
global GP_METHOD
rotz = @(alpha) [cos(alpha) -sin(alpha) 0; ...
                sin(alpha)  cos(alpha) 0; ...
                    0           0       1];
covfcn = {'covSum', {'covSEiso', 'covNoise'}};
J = [];

t_gprf = [];
t_libgp = [];
fig_handles = {};
for i = 1:length(Paths)
    ts_gprf = tic;
    path = Paths{i};
    vels = Vels{i};
    
    % predicted velocities
    path_local = rotz(start(3))'*(path - start);
    
    % method 1: using libgp
    if (strcmp(GP_METHOD, 'LIBGP'))
        ts_libgp = tic;
%         [Vx_PRED, Vx_VAR] = GPRF.libgp_x.predictWithVariance(path_local(1:2, :));
        Vx_PRED = GPRF.libgp_x.predict(path_local(1:2, :));
        t_libgp(end+1) = toc(ts_libgp);
        ts_libgp = tic;
%         [Vy_PRED, Vy_VAR] = GPRF.libgp_y.predictWithVariance(path_local(1:2, :));
        Vy_PRED = GPRF.libgp_y.predict(path_local(1:2, :));
        t_libgp(end+1) = toc(ts_libgp);
%         Vx_SD = sqrt(Vx_VAR);
%         Vy_SD = sqrt(Vy_VAR);
        v_pred = [-Vy_PRED; Vx_PRED];
    elseif(strcmp(GP_METHOD, 'GPML'))
        % method 2: using gpml
        ts_libgp = tic;
        [Vx_PRED, Vx_SD, ~, ~] = gp(cf_options.hypx, cf_options.inffcn, cf_options.meanfcn, covfcn, cf_options.likfcn, GPRF.state', GPRF.vel(1, :)', path_local(1:2, :)');
        t_libgp(end+1) = toc(ts_libgp);
        ts_libgp = tic;
        [Vy_PRED, Vy_SD, ~, ~] = gp(cf_options.hypy, cf_options.inffcn, cf_options.meanfcn, covfcn, cf_options.likfcn, GPRF.state', GPRF.vel(2, :)', path_local(1:2, :)');
        t_libgp(end+1) = toc(ts_libgp);
        v_pred = [-Vy_PRED'; Vx_PRED'];
    end
    % compute actual velocity of trajectories
    dist = diff(path_local,1, 2);
    v_actual = [-vels(2, :); vels(1, :)];
    v_actual_normed = vectorNormalized(v_actual);  % normalize the actual vector
    
    v_pred_norm = sqrt(sum(v_pred.^2, 1));
    v_pred_normed = v_pred./v_pred_norm;
    zero_index = find(v_pred_norm < 0.01);
    
    
    reward = dot(v_pred_normed, v_actual_normed);    
    cost = reward*-1;
    cost(zero_index) = 10;          % set the cost of unexplored areas to an arbitrarily high number
    J(end+1) = sum(cost);
    t_gprf(end+1) = toc(ts_gprf);
    
    % PLOT
   
    subplot(1, 2, 1); hold on; 
    subplot(1, 2, 2); hold on; 
    fig_handles{end+1} = plot(-path_local(2, :), path_local(1, :), '.-', 'Color', [0.7, 0.7, 0.7]);
    fig_handles{end+1} = quiver(-path_local(2, :), path_local(1, :), v_actual(1, :), v_actual(2, :), 'Color', [0.7, 0.7, 1, 0.3]);
    fig_handles{end+1} = quiver(-path_local(2, :), path_local(1, :), v_pred(1, :), v_pred(2, :), 'Color', [1, 0.7, 0.7, 0.3]);
    pause(0.03)
end

for i = 1:length(fig_handles)
    delete(fig_handles{i})
end

% disp(['CF Timing: Per cf eval: ', num2str(mean(t_gprf)), ' Per prediction call: ', num2str(mean(t_libgp))]);
    
end

function v_normed = vectorNormalized(v)
norm = sqrt(sum(v.^2, 1));
v_normed = v./norm;
% v_normed(find(isnan(v_normed))) = 0;
v_normed(:, find(norm < 0.00001)) = -10;
end
