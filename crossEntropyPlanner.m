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

%% Implementation of Cross-Entropy Randomized Motion Planning by Marin Kobilarov (RSS 2011)
% Note: I don't have access to the Statistics and Machine Learning Toolbox
% in matlab, so a lot of these functions are written by me

function ce_plan = crossEntropyPlanner(traj_options, ce_options, cf_options, obst, start, goal, GPRF, seed)
% trajectory parameters
global omega_vector vel_vector duration_vector
m = traj_options.num_mp;
N = traj_options.num_samples;
param = traj_options.param;

omega_vector = linspace(-traj_options.omega_bd, traj_options.omega_bd, 1000);

if strcmp(param, 'velocity')
    T = traj_options.T;                                % duration [s] -- fixed
    vel_vector = linspace(0, traj_options.vel_max, 50);
    
elseif strcmp(param, 'duration')
    vel = traj_options.vel;                            % velocity [m/s] -- fixed
    duration_vector = linspace(0, traj_options.duration_max, 50);
end


% iteration parameters 
MAX_ITER = ce_options.MAX_ITER;                      % number of iterations
K = ce_options.K;                              % number of GMM components: K=1 is a single gaussian

% sampling parameters
lambda = ce_options.lambda;                         % cost smoothing rate
nu = ce_options.nu;                          % some slight noise to prevent degeneracy
rho = ce_options.rho;                          % percent quantile to truncate the cost
r_dev = ce_options.r_dev;                          % deviation of resampling based on covariance; usually r ~ N(0, 1)

dt = ce_options.dt;                           % for plotting & cost function evaluation

%% CE Motion Planning 

% 1. Choose initial trajectory samples Z1, ..., ZN, Set j = 0, hat_gamma(0) = inf
% 2. for each step j
%   2a. Generate elite set
%   2b. compute hat_v(j) = argmin_{v \in V} 1/ Ej \sum ln p(Zk, v):
%       if k = 1: mu, sigma analytically computed
%       if k >= 2: EM
%   2c. Generate samples Z1, ..., ZN from p( , v(j)) and compute the rho-th
%       quantile
%   2d. Termination criterion
%
% Data Structures
%
% mu{k}(iter, 2*m)              A cell array with k cells, iter rows and 2*m columns.
% phi(iter, k)                  GMM weight matrix with sum(phi, 1) sum across rows = 1.
% Sigma{k}                      The covariance of each GMM at each iter. I dont store all of
%                               the covariance matrices over all iters.
% Z(N, 2*m)                     Parameter set at each step
% Z_elite(num_elite, 2*m)
% W(num_elite, k)               Membership matrix for each elite trajectory


%% 1. Choose initial trajectory samples
t_resample = [];
t_elite = [];
t_cf = [];
t_em = [];
t_cost_eval = [];
t_iter = [];

ts_all = tic;
    
Z = [];     % parameter set with structure [omegas, durations]
Paths = {};  
Vels = {};

ts_resample = tic;
% generate n samples
[Z, Paths, Vels] = sampleRandomTrajectories('seed_random', start, obst, ce_options, traj_options, [], {}, {}, seed);
t_resample(end+1) = toc(ts_resample);


%% 2. Iterate

j = 1;
hat_gamma(1) = inf;

% initialize GMMs
for k = 1:K
    Sigma{k} = cov(Z);
    
    % randomly initialize!
    omegas = datasample(omega_vector, m);
    if strcmp(param, 'velocity') velocities = datasample(vel_vector, m); 
    elseif strcmp(param, 'duration') durations = datasample(duration_vector, m);
    end
    
    if strcmp(param, 'velocity') mu{k}(j,:) = [omegas, velocities];
    elseif strcmp(param, 'duration') mu{k}(j,:) = [omegas, durations];
    end

end
phi(j,:) = ones(1, K)*1/K;


while j <= MAX_ITER
%     disp(['CE Planner: iter ', num2str(j)]);
    
    ts_iter = tic;
    
    wpts_elite = {};    % wpts of the elite traj; store for plotting
    Z_elite = [];         % param of the elite traj; store for plotting
    J_vec = [];
    num_elite = 0;
    
    % --------------------------------------------------------------------
    % Generate Z_elite set
    % --------------------------------------------------------------------
    ts_cf = tic;
    
    % evaluate using CE options
    if strcmp(ce_options.cost_fcn, 'GOAL')
        J_vec = costFunctionGoal(Z, Paths, goal, lambda);
    elseif strcmp(ce_options.cost_fcn, 'GPRF')
        J_vec = costFunctionGPRF(cf_options, ce_options.dt, Z, Paths, Vels, GPRF, start);
    end
%     J_vec = computeObjective(cf_type, cf_options, traj, wpts)
    t_cf(end+1) = toc(ts_cf);
    
    ts_elite = tic;
    while (num_elite < 2)
        for n = 1:N
            J = J_vec(n);
            if J <= hat_gamma(j)
                Z_elite(end+1, :) = Z(n, :);
                wpts_elite{end+1} = Paths{n};              
                num_elite = num_elite + 1;
            end
        end
        % if elite set has less than two trajectories, allow increase in cost from nominal by 5%
        if (num_elite < 2) 
            hat_gamma(j) = hat_gamma(j)*1.05; 
        end
    end
    t_elite(end+1) = toc(ts_elite);
    
    % --------------------------------------------------------------------
    % update hat_gamma
    % --------------------------------------------------------------------
    
    [J_sorted, J_idx] = sort(J_vec);
    hat_gamma(j+1) = J_sorted(ceil(rho*N));
    
    % --------------------------------------------------------------------
    % Compute hat_v(j) = argmin_{v \in V} 1/ Ej \sum ln p(Zk, v)
    % --------------------------------------------------------------------
    
    ts_em = tic;
    if K == 1                   % Single Gaussian Component
       % update mu
       mu{K}(j+1, :) = 1/num_elite * sum(Z_elite, 1);
       
       % update sigma
       sigma_unnormalized = zeros(2*m);
       for i = 1:num_elite
        sigma_unnormalized = sigma_unnormalized +(Z_elite(i, :) - mu{K}(j+1,:))'*(Z_elite(i, :) - mu{K}(j+1, :));
       end
       sigma = 1/num_elite * sigma_unnormalized;
       
       % inject some noise to prevent degeneracy
       Sigma{k} = sigma + diag(ones(1,2*m) * nu);
       
    elseif K >= 2               % Multimodal
        % Do EM!
        W = [];
        % E-step: determine membership
        for i = 1:num_elite
            z = Z_elite(i, :); 
            
            for k = 1:K
                p(k) =  prob(z, mu{k}(j,:), Sigma{k})*phi(j,k);
            end
            W(i, :) = p/sum(p);
        end
        
        % M step: update mean and covariance
        for k = 1:K
            phi(j+1, k) = sum(W(:, k))/num_elite;
            mu{k}(j+1,:) = W(:, k)'* Z_elite / sum(W(:, k));
            Sigma_unnormalized = 0;
            for i = 1:num_elite
                 Sigma_unnormalized = Sigma_unnormalized + W(i, k)*(Z_elite(i, :) - mu{k}(j+1,:))'*(Z_elite(i, :) - mu{k}(j+1,:)); 
            end
            Sigma{k} = Sigma_unnormalized / sum( W(:, k) );
            
            % inject some noise to prevent degeneracy
            Sigma{k} = Sigma{k} + diag(ones(1, 2*m) * nu);
        end
    end
    t_em(end+1) = toc(ts_em);
    
    % --------------------------------------------------------------------
    % Generate samples Z1, ..., ZN from p( , v(j))
    % --------------------------------------------------------------------
    
    ts_resample = tic;
   
    for k = 1:K
        A{k} = sqrtm(Sigma{k}); % compute Ak = sqrt(sigma_k)
    end

    % generate N samples: 
    [Z, Paths, Vels] = sampleRandomTrajectories('gmm', start, obst, ce_options, traj_options, phi, mu, A, struct());
    
    t_resample(end+1) = toc(ts_resample);

    j = j+1;
    
    t_iter(end+1) = toc(ts_iter);
%     disp(['CE-Planner timing: Total: ', num2str(t_iter(end)), ' t_cost_eval:', num2str(t_cf(end)), ' t_elite selection: ', num2str(t_elite(end)), ' t_resample: ', num2str(t_resample(end))]);
end

tf_all = toc(ts_all);
disp(['CE-Planner: Total(', num2str(MAX_ITER), ')/Per iter: ', num2str(tf_all), '/', num2str(tf_all/MAX_ITER)]); 

%% Package elite trajectories and output the nominal one
GMM_mu = [];
for k = 1:K
    GMM_mu(end+1, :) = mu{k}(end, :);
end

ce_plan = struct('elites', Z_elite, ...
                 'GMM_mu', GMM_mu, ...
                 'GMM_Sigma', Sigma, ...
                 'best', Z(J_idx, :)); 
             
