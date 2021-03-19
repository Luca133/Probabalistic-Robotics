%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Probabilistic Robotics Assignment
% Task 2: Mapping Part 1
% By Luca J. Ricagni
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear
close all   % Close all figures
clear       % Clear all variables in workspace
clc         % Clear command window

%% Import relevant data files

ground_truth_pose = load('ground_truth_pose.dat');      % Import ground truth pose
noisy_pose_estimate = load('noisy_pose_estimate.dat');  % Import noisy pose estimates


%% Declare fiducial locations (x,y) coords
fiducial_locations = [3 2.2;        % Fuducial 1
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

%% Plot

figure(1)                                                       % Open figure for plotting
hold on                                                         % Keep figure open
plot(ground_truth_pose(:,1), ground_truth_pose(:,2), 'k')       % Plot ground truth pose in black
plot(noisy_pose_estimate(:,1), noisy_pose_estimate(:,2), 'r')   % Plot noisy pose estimate in red
plot(fiducial_locations(:,1), fiducial_locations(:,2), 'ob')    % Plot fiducial landmarks locations

title('Graph showing the ground truth, filtered estimate and the fiducial marker locations') % Add title to plot
legend('ground truth', 'noisy estimate', 'fiducial marker locations') % Add key to graph

xlabel('X coordinate') % Add label to x axis
ylabel('Y coordinate') % Add label to y axis


