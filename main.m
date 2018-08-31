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

%% An implementation of the FLUID planner in MATLAB.
% This implementation is not complete and requires cleanup. Please use the
% C++ version of this planner if possible. 
% TODO(xuning@cmu.edu): 
%   - Either clean up the matlab implementation, or
%   - Write a wrapper around the C++ implementation.

clc
clear all
close all

%% Loads Odometry Data
% Function `loadData` is not specified here, but feel free to adapt for
% your own use.

loadData; % also loads: N, dt, mass

global GP_METHOD
GP_METHOD = 'LIBGP'; % one of: { 'GPML', 'LIBGP' }
disp(['Running CE Planning with ', GP_METHOD, ' predictor']);

dT = 3;                     % train over 5s of data
dN = ceil(dT/dt);
dT_incr = 0.3;                % collect trajectory every 1 s
dN_incr = ceil(dT_incr/dt);

%% Obstacle parameters 
% Assuming obstacles are circles.

road_width = 300;
num_obst = 50;
road_length = 600;
radius = 6;

% Obstacles in the form of circles [xc, yc, r]; if any. else, set obst to [] 
obst = [7.5, -61, 6;
        34, -47, 6;
        -15, 10, 3];

obst_plot = obst;
% obst_plot = [-obst(:, 2), obst(:, 1), obst(:, 3)];   % in local frame
[num_obst, num_circ_params] = size(obst);
assert(num_circ_params == 3);

subplot(1, 2, 1); hold on; plotObstacles(obst_plot);

pause(0.1);

%% Parameters

covfcn = {'covSum', {'covSEiso', 'covNoise'}};
param = 'velocity';                 % or 'duration';
m = 2;                              % number of segments 
traj_options = struct('param', 'velocity', ...
                      'num_mp', m, ...        % m
                      'num_samples', 50, ... % N
                      'omega_bd', 2, ...
                      'vel_max', 16, ...
                      'duration_max', 5, ...
                      'vel', 1, ...
                      'T', dT/m);
ce_options = struct('MAX_ITER', 15, ...
                    'K', 1, ...             % num gaussian components 
                    'lambda', 1, ...        % cost smoothing rate
                    'nu', 0.01, ...         % slight noise to prevent degeneracy
                    'rho', 0.3, ...         % percent quantile to truncate the cost
                    'r_dev', 1, ...         % deviation of resampling based on covariance; usually r ~ N(0, 1)
                    'dt', 0.1, ...          % for plotting & cost function evaluation
                    'cost_fcn', 'GOAL');    % cost function type one of: { GPRF, GOAL }

if strcmp(ce_options.cost_fcn, 'GPRF')
    cf_options = struct('y_ub', 10, ...     % max y of the patch size: 10m
                        'y_lb', -10, ...    % min y of the patch size: -10m
                        'x_ub', 50, ...     % max x of the patch size
                        'x_lb', -10, ...    % min x of the patch size
                        'num_test', 31,...      % size of the local intent map grid
                        'meanfcn', [], ...
                        'likfcn', @likGauss, ...
                        'inffcn', @infGaussLik, ...
                        'hypx', struct('mean', [], 'cov', [1.2, 4.1, 4.0], 'lik', -1), ...
                        'hypy', struct('mean', [], 'cov', [1.0, 1.5, 4.0], 'lik', -1), ...
                        'window', 6);
end

traj_window_world = {};
state_window_local = {};
vel_window_local = {};

x_vec = linspace(cf_options.x_lb, cf_options.x_ub, cf_options.num_test);
y_vec = linspace(cf_options.y_lb, cf_options.y_ub, cf_options.num_test);
[X_quiv,Y_quiv] = meshgrid(x_vec, y_vec);

libgp_x = mex_interface(str2fun('libgp_mex'));
libgp_y = mex_interface(str2fun('libgp_mex'));
covfcn2 =  'CovSum( CovSEiso, CovNoise)';
libgp_x.initialize(2, covfcn2, cf_options.hypx.cov);
libgp_y.initialize(2, covfcn2, cf_options.hypy.cov);
GPRF = struct('libgp_x', libgp_x, 'libgp_y', libgp_y);
seed = struct('mu', [], 'A', []);


for n = dN:dN_incr:N 
    disp(['At time ', num2str(dt*n), 's:']);
    fig_handles = {};
    
    %% Consolidate window of trajectories and learn GPRF
    disp(['Learning GPRF...']);
    tic;
    % Find prev segment.
    n_prev = n-dN+1;
    prev_state = traj(:, n_prev);
    t = data.t(n_prev);
    Rz = rotz(prev_state(3));               % extract rotation at this point
    
    % Sore traj/state/vel in body frame & plot.
    traj_seg_local = Rz'*(traj(:, n_prev:n) - prev_state);
    state_seg_local = traj_seg_local(1:2, :);
    vel_seg_local = [];
    for i = n_prev:n
        vel_seg_local(:, end+1) = Rz'*rotz(state.heading(i))*[state.velocity(i); 0; 0];
        subplot(1, 2, 1); fig_handles{end+1} = scatter(state.X(i), state.Y(i), 10, 'c', 'filled');
        
    end
    
    % Add to set of trajectories to learn over.
    if length(state_window_local) >= cf_options.window
        state_window_local(1) = [];
        vel_window_local(1) = [];
    end
    state_window_local{end+1} = state_seg_local;
    vel_window_local{end+1} = vel_seg_local;
    
    subplot(1, 2, 2); hold on; xlabel('Y'); ylabel('X'); title(['Time ', num2str(t), 's using a window of ', num2str(length(state_window_local)), ' trajectories']);
    [GPRF, fig_handles_2] = learnGPRF(cf_options, state_window_local, vel_window_local, GPRF);
    toc
    
    %% Plan using CE
    disp(['Running CE planner...']);
    tic;
    start_n = traj(:, n);
    subplot(1, 2, 1);  hold on; scatter(start_n(1), start_n(2),  40, 'g', 'filled');
    if strcmp(ce_options.cost_fcn, 'GOAL')
        goal_n = traj(:, n+dN);
    else
        goal_n = [];                        % not used for GPRF
    end
    
    seg_plan = crossEntropyPlanner(traj_options, ce_options, cf_options, obst, start_n, goal_n, GPRF, seed);
    toc
    
    %% Plot
    disp(['Plotting...']);
    [num_elites, ~] = size(seg_plan.elites);
    for i = 1:num_elites
        z = seg_plan.elites(i, :);
        [wpts, vels, start_pts] = generateTrajectory_constant_T( z(m+1:m+m), z(1:m), traj_options.T, ce_options.dt, start_n);
        subplot(1, 2, 1); hold on; plot(wpts(1, :), wpts(2, :), 'Color', [0.7, 0.7, 0.7, 0.3]); 
        
        wpts_local = rotz(start_n(3))'*(wpts - start_n);
        start_pts_local = rotz(start_n(3))'*(start_pts - start_n);
        fig_handles{end+1} = subplot(1, 2, 2); hold on; plot(-wpts_local(2, :), wpts_local(1, :), 'Color', [0.7, 0.7, 0.7, 0.3]);
        fig_handles{end+1} = subplot(1, 2, 2); hold on; scatter(-start_pts_local(2, 2:end), start_pts_local(1, 2:end), 10, [0.7, 0.7, 0.7], 'filled');
        pause(0.001);
    end

    % seed the next set of trajectories
    seed = struct('mu', seg_plan.GMM_mu, 'A', sqrtm(seg_plan.GMM_Sigma));
    pause(0.001)
    
    for i = 1:length(fig_handles)
        delete(fig_handles{i})
    end
    for i = 1:length(fig_handles_2)
        delete(fig_handles_2{i})
    end
    disp(['end of iteration.']);
    
end
