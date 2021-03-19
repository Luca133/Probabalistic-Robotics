%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Probabilistic Robotics Assignment
% Task 4: Implement a Particle Filter
% By Luca J. Ricagni
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear
close all   % Close all figures
clear       % Clear all variables in workspace
clc         % Clear command window

%% Import relevant data files

ground_truth_pose = load('ground_truth_pose.dat');      % Import ground truth pose
noisy_pose_estimate = load('noisy_pose_estimate.dat');  % Import noisy pose estimates
motor_commands = load('motor_commands.dat');  % Import motor commands
fiducials = load('fiducials.dat');  % Import fiducials
fiducials_noise = load('fiducials_noise.dat');  % Import noisy fiducials
ranges = load('ranges.dat');  % Import ranges

%% Declare variables
filtered_estimate = zeros(2000, 4); % Place to store filtered position estimates

numOfIterations = 2000; % Number of times the loop will run

numOfFiducials = 12;

M = 1000; % Number of particles

X_t = zeros(M, 4); % Initialise particle storage

% Initialise particles to starting groung truth pose
for i=1:M
    X_t(i,1:3) = ground_truth_pose(1,:);
end

%% Declare map (m) which is list of fiducials
map = [3 2.2;        % Fuducial 1
    3 0;          % Fuducial 2
    3 -2.2;       % Fuducial 3
    1 -2.2;       % Fuducial 4
    -1 -2.2;      % Fuducial 5
    -3 -2.2;      % Fuducial 6
    -4.5 -2.2;    % Fuducial 7
    -4.5 0;       % Fuducial 8
    -4.5 2.2;     % Fuducial 9
    -3 2.2;       % Fuducial 10
    -1 2.2;       % Fuducial 11
    1 2.2];       % Fuducial 12

mult = 0;
for i=1:numOfIterations
    % Initialise array to hold fiducials for time t
    observed_fiducials = zeros(numOfFiducials,3);
    
    % Extract the 12 fiducial values for a given time t
    for j=1:numOfFiducials
        observed_fiducials(j,:) = fiducials(mult*12 + j,:);
    end
    
    mult = mult + 1;
    
    % Call monte_carlo_localization (particle filter)
    X_t = monte_carlo_localization(X_t, motor_commands(i,:), observed_fiducials, map, M);
   
    %% Take mean (x,y) of particles
    x_mean = mean(X_t(:,1));
    y_mean = mean(X_t(:,2));
    
    filtered_estimate(i,1) = x_mean;
    filtered_estimate(i,2) = y_mean;
end

%% Plot
figure(1)
hold on

plot(filtered_estimate(:,1), filtered_estimate(:,2), 'b')       % Plot filtered estimate
plot(ground_truth_pose(:,1), ground_truth_pose(:,2), 'k')       % Plot ground truth pose in black
plot(noisy_pose_estimate(:,1), noisy_pose_estimate(:,2), 'r')   % Plot noisy pose estimate in red

title('Graph showing the filtered estimate, ground truth and the noisy estimate') % Add title to graph
legend('filtered estimate', 'ground truth', 'noisy estimate') % Add key to graph

xlabel('X coordinate') % Add label to x axis
ylabel('Y coordinate') % Add label to y axis


%% Calculate L2Norm distances
% Initialise arrays to store calculated values
l2Norm_ground_truth_noisy_estimate = zeros(numOfIterations,1);
l2Norm_ground_truth_filtered_estimate = zeros(numOfIterations,1);

% loop to iterate through and calculate the L2Norm values for each time step
for p=1:numOfIterations
    l2Norm_ground_truth_noisy_estimate(p,1) = sqrt((noisy_pose_estimate(p,1) - (ground_truth_pose(p,1)))^2 + (noisy_pose_estimate(p,2) - (ground_truth_pose(p,2)))^2);
    
    l2Norm_ground_truth_filtered_estimate(p,1) = sqrt((filtered_estimate(p,1) - (ground_truth_pose(p,1)))^2 + (filtered_estimate(p,2) - (ground_truth_pose(p,2)))^2);
end

%% Plot L2Norm distances
figure(2)
hold on
plot(l2Norm_ground_truth_noisy_estimate, 'k')

plot(l2Norm_ground_truth_filtered_estimate, 'b')

title('Graph showing the L2Norm distance between ground truth and noisy pose, with ground truth and filtered estimate') % Add title to graph
legend('L2Norm: ground truth - noisy estimate', 'L2Norm: ground truth - filtered estimate') % Add key to graph

xlabel('Time step (t)') % Add label to x axis
ylabel('L2Norm distance') % Add label to y axis



