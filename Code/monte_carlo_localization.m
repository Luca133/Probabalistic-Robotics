%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Probabilistic Robotics Assignment
% Task 4: Implement a Particle Filter
% Page 252 Probabilistic Robotics By S. Thrun
% By Luca J. Ricagni
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arguments:
% X_t_previous - set of previous particles
% u_t - control/motor commands
% z_t - set of features visible, a 12x3 matrix as the number of fiducials in the map is 12 therefor many will be 0, each row is a (range,bearing,signature) of a fiducial% m - map
% M - number of particles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Outputs:
% X_t - new set of particles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X_t] = monte_carlo_localization(X_t_previous, u_t, z_t, map, M)

delta_t = 0.1; % Declare time step

% Inilialise variables
X_t = zeros(M, 4);
X_t_hat = X_t;

w_t = zeros(M, 1);

%% Update particles using motion and landmark models
for m=1:M
    % Use particles from current belief and sample from the motion model
    X_t(m,1:3) = sample_velocity_motion_model(u_t, X_t_previous(m,:), delta_t);
    
    % Determine importance weight of particle
    w_t(m) = landmark_model_known_correspondence(z_t, X_t(m,:), map);

    % Store particle pose and weight
    X_t_hat(m,:) = X_t_hat(m,:) + X_t(m,:);
    X_t_hat(m,4) = w_t(m);
end

%% Perform roulette wheel selection
for m = 1:1:M
    
    % Randomly generate selection point which is not 0
    selection_point = 0;
    while selection_point == 0
        selection_point = sum(X_t_hat(:,4)) * rand(1);
    end
    
    % Iterate through until sum of weights so far > selection point
    running_total = 0;
    roulette_counter = 1;
    while (running_total < selection_point)
        running_total = running_total + X_t_hat(roulette_counter,4);
        roulette_counter = roulette_counter + 1;
    end
    % Store particle
    X_t(m,:) = X_t_hat(roulette_counter-1,:);
end

end




