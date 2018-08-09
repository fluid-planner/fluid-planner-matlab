function [Z, Paths, Vels] = sampleRandomTrajectories(sample_strategy, start, obst, ce_options, traj_options, phi, mu, A, seed)
global omega_vector vel_vector duration_vector

dt = ce_options.dt;
K = ce_options.K;
r_dev = ce_options.r_dev;
T = traj_options.T;
vel = traj_options.vel;
param = traj_options.param;
N = traj_options.num_samples;
m = traj_options.num_mp;

Z = [];
Paths = {};
Vels = {};
n = 1;

while n <= N
    
    if strcmp(sample_strategy, 'gmm')
        
        if K == 1
            k = 1;
        else
            k = discretesample(gmm.phi(end,:), 1);      % Choose k \in {1:K} proportional to phi(k)
        end
        
        r = -r_dev + (r_dev*2)*rand(1, 2*m);    % sample r ~ N(0, 1) and 
        z = mu{k}(end,:) + (A{k}*r')';             % set Zi = mu + Ak*r 
        z(m+1:m+m) = max(0,z(m+1:m+m));         % safety: need to lower bound the times to zero...  
        
        if strcmp(param, 'velocity') 
            [wpts, vels] = generateTrajectory_constant_T( z(m+1:m+m), z(1:m), T, dt, start);
        elseif strcmp(param, 'duration') 
            wpts = generateTrajectory_constant_v(vel, z(1:m), z(m+1:m+m), dt, start);
        end
        
    elseif strcmp(sample_strategy, 'random')
        omegas = datasample(omega_vector, m);
        
        if strcmp(param, 'velocity') 
            velocities = datasample(vel_vector, m); 
            [wpts, vels] = generateTrajectory_constant_T(velocities, omegas, T, dt, start);
            z = [omegas, velocities];
        elseif strcmp(param, 'duration') 
            durations = datasample(duration_vector, m);
            wpts = generateTrajectory_constant_v(vel, omegas, durations, dt, start);
            z = [omegas, durations];
        end
    elseif strcmp(sample_strategy, 'seed_random')
        
        omegas = datasample(omega_vector, m);
        
        if strcmp(param, 'velocity') 
            
            if (isempty(seed.mu))
                velocities = datasample(vel_vector, m);
                z = [omegas, velocities];
            else
            
                % replace the first segment with the seeded
                assert(length(seed.mu) == 2*m)

                r = -r_dev + (r_dev*2)*rand(1, 2*m);    % sample r ~ N(0, 1) and 
                z = seed.mu + (seed.A*r')';             % set Zi = mu + Ak*r 

                omegas = z(1:m);
                velocities = z(m+1:m+m);
            end
            
            % generate trajectories
            [wpts, vels] = generateTrajectory_constant_T(velocities, omegas, T, dt, start);
            
%         elseif strcmp(param, 'duration') 
%             durations = datasample(duration_vector, m);
%             wpts = generateTrajectory_constant_v(vel, omegas, durations, dt, start);
%             z = [omegas, durations];
        end
    
    elseif strcmp(sample_strategy, 'halfnhalf')
        
        % half randomized
        if n < ceil(N/2)
            omegas = datasample(omega_vector, m);

            if strcmp(param, 'velocity') 
                % replace the first segment with the seeded
                assert(length(seed.mu) == 2*m)

                r = -r_dev + (r_dev*2)*rand(1, 2*m);    % sample r ~ N(0, 1) and 
                z = seed.mu + (seed.A*r')';             % set Zi = mu + Ak*r 

                omegas = z(1:m);
                velocities = z(m+1:m+m);
                
                % generate trajectories
                [wpts, vels] = generateTrajectory_constant_T(velocities, omegas, T, dt, start);
                
%             elseif strcmp(param, 'duration') 
%                 durations = datasample(duration_vector, m);
%                 wpts = generateTrajectory_constant_v(vel, omegas, durations, dt, start);
%                 z = [omegas, durations];
            end
        else % half seeded
            if K == 1
            k = 1;
            else
                k = discretesample(gmm.phi(end,:), 1);      % Choose k \in {1:K} proportional to phi(k)
            end

            r = -r_dev + (r_dev*2)*rand(1, 2*m);    % sample r ~ N(0, 1) and 
            z = mu{k}(end,:) + (A{k}*r')';             % set Zi = mu + Ak*r 
            z(m+1:m+m) = max(0,z(m+1:m+m));         % safety: need to lower bound the times to zero...  
            
            if strcmp(param, 'velocity') 
                [wpts, vels] = generateTrajectory_constant_T( z(m+1:m+m), z(1:m), T, dt, start);
            elseif strcmp(param, 'duration') 
                wpts = generateTrajectory_constant_v(vel, z(1:m), z(m+1:m+m), dt, start);
            end
        end
            
    end
    
    
    if isTrajectorySafe(wpts, obst)
        Z(n, :) = z;
        Paths{n} = wpts;
        Vels{n} = vels;
        n = n+1;
    end
end
    
end