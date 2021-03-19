%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Probabilistic Robotics Assignment
% Task 2: Mapping Part 2
% By Luca J. Ricagni
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear
close all   % Close all figures
clear       % Clear all variables in workspace
clc         % Clear command window

%% Delcare variables
prior = 0.5;                    % no knowledge of occupancy
free  = 0.05;			% probability of free space
occ   = 0.95;			% probability of occupancy


scale = 0.05;                   % size of each cell in m
Env_size = 20;                  % size of environment in m (assume square for now)
l_free = log(free/(1-free));    % log odds of free cell
l_occ  = log(occ/(1-occ));      % log odds of occupied cell
l_prior =log(prior/(1-prior));  % log-odds of prior

dimx = round(Env_size/scale);   % find number of cells in x-dimension
dimy = dimx;                    % same number of cells in y-dimension

% initialise map to prior occupancy
map = prior*ones(dimx,dimy);

%initial pose of robot in the map
start_pose = [200 200 0];

%% Import relevant data files

ground_truth_pose = load('ground_truth_pose.dat');      % Import ground truth pose
ranges = load('ranges.dat');                            % Import ranges

%% Convert to cell scale
numits = length(ground_truth_pose);

ground_truth_pose(:,1)=ground_truth_pose(:,1)/scale; % convert x to cell scale
ground_truth_pose(:,2)=ground_truth_pose(:,2)/scale; % convert y to cell scale

figure(1)

%% step through data set, update map and display
for(x=1:numits)
    pose=ground_truth_pose(x,:) + start_pose;
    x
    map = add_robot(pose, map, scale, l_free, l_occ, l_prior, ranges(x,:));
    image([1 dimx], [1 dimy], repmat((1-map)', [1 1 3]));
    set(gca, 'ydir', 'normal')
    xlabel('x-axis');
    ylabel('y-axis');
    drawnow
    pause(0.01)
end

title('Occupancy grid map') % Add title to graph
xlabel('X coordinate') % Add label to x axis
ylabel('Y coordinate') % Add label to y axis

