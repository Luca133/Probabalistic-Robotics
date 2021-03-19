%% Task 2, Figure 2,
% Generate an occupancy grid map using the ground truth pose and range
% finder readings from the data set provided
%% Clear figures, workspace and command window
clf;
clear;
clc;

%% Load Data
Data_GroundTruth = load("ground_truth_pose.dat");           %Load GroundTruth Data
Data_Ranges = load ("ranges.dat");	%

%% Figure 2

prior = 0.5;                    % no knowledge of occupancy
free  = 0.3;			% probability of free space
occ   = 0.9;			% probability of occupancy

scale = 0.01;                   % size of each cell in m
Env_size = 20;                  % size of environment in m (assume square for now)
l_free = log(free/(1-free));    % log odds of free cell
l_occ  = log(occ/(1-occ));      % log odds of occupied cell
l_prior =log(prior/(1-prior));  % log-odds of prior

dimx = round(Env_size/scale);   % find number of cells in x-dimension
dimy = dimx;                    % same number of cells in y-dimension

% initialise map to prior occupancy
map = prior*ones(dimx,dimy);

StartingPose = [Env_size/(2 * scale), Env_size/(2 * scale), 0]; %Centre of the map

numits = length(Data_GroundTruth);

odometry = Data_GroundTruth;

odometry(:,1)=odometry(:,1)/scale; % convert x to cell scale
odometry(:,2)=odometry(:,2)/scale; % convert y to cell scale

%step through data set, update map and display
for(Index=1:numits)
    Index
    pose = odometry(Index,:) + StartingPose;
    map = add_robot(pose, map, scale, l_free, l_occ, l_prior, Data_Ranges(Index,:));
end

figure(3)
title("Occupancy Grid Map graph of Robot in Warehouse");
xlabel("X Axis Distance (m)");
ylabel("Y Axis Distance (m)");
image([1 dimx], [1 dimy], repmat((1-temp_map)', [1 1 3]));
set(gca, 'ydir', 'normal')
drawnow





