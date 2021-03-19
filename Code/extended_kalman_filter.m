%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Probabilistic Robotics Assignment
% Task 3: Extended Kalman Filter algorithm
% Page 204 Probabilistic Robotics By S. Thrun
% By Luca J. Ricagni
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arguments:
% mew_t_previous - mean guassian estimate of robot pose
% sigma_t_previous - co-variance of robot pose
% u_t - control/motor commands
% z_t - set of features visible, a 12x3 matrix as the number of fiducials in the map is 12 therefor many will be 0, each row is a (range,bearing,signature) of a fiducial
% c_t - correspondence variables
% m - map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Outputs:
% mew_t - new estimate of mean of robot pose
% sigma_t- new estimate of co-variance of robot pose
% p_zt - likelihood of the feature observation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mew_t, sigma_t, p_zt] = extended_kalman_filter(mew_t_previous, sigma_t_previous, u_t, z_t, c_t, m)
%% Declare and extract variables
delta_t = 0.1; % Declare time interval change

alpha1 = 0.1;       % noise measured in rotation caused by rotation
alpha2 = 0.02;      % noise measured in rotation caused by translation
alpha3 = 0.2;       % noise measured in translation caused by translation
alpha4 = 0.01;      % noise measured in translation caused by rotation

theta = mew_t_previous(3); % Extract the angle from the pose

v_t = u_t(1); % Extract linear velocity from motor command

w_t = u_t(2); % Extract angular velocity from motor command

if w_t == 0
    w_t = (rand-0.5)*0.0001; % add noise to w if w == 0
end

I = eye(3); % Declare identity matrix

%% Compute Jacobians needed for linearised motion model
% G_t is the derivative of the function g with respect to x_t-1 evaluated at u_t and mew_t-1
G_t = [1 0 (-(v_t/w_t)*cos(theta) + (v_t/w_t)*cos(theta + w_t*delta_t))
       0 1 (-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_t))
       0 0 1];
   

% V_t is the derivative of the function g with respect to the motion parameters evaluated at u_t and mew_t-1
V_t = [((-sin(theta) + sin(theta + w_t*delta_t))/w_t) (((v_t*(sin(theta) - sin(theta + w_t*delta_t)))/w_t^2) + (v_t*(cos(theta + w_t*delta_t))*delta_t)/w_t)
       ((cos(theta) - cos(theta + w_t*delta_t))/w_t) (-(v_t*(cos(theta) - cos(theta + w_t*delta_t))/w_t^2) + ((v_t*(sin(theta + w_t*delta_t))*delta_t)/w_t))
       0                                                                                    delta_t];
   
   
%% Determine the motion noise covariance matrix
%M_t is the covariance matrix of the noise in control space
M_t = [(alpha1*(v_t^2) + alpha2*(w_t^2)) 0
       0 (alpha3*(v_t^2) + alpha4*(w_t^2))];
       
%% Calculate motion update
% Calculate new estimate of pose based off previous estimate and the new motor commands v_t and w_t
mew_hat_t = mew_t_previous + [(-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_t)) ((v_t/w_t)*cos(theta) - (v_t/w_t)*cos(theta + w_t*delta_t)) w_t*delta_t];

if mew_hat_t(3) > pi || mew_hat_t(3) < -pi
    mew_hat_t(3) = wrapToPi(mew_hat_t(3));
end

% Calculate new estimate of covariance of position
% V_t*M_t*(V_t)^transpose provides the approximate mapping between motion noise in comtrol space and motion noise in state space
sigma_hat_t = G_t*sigma_t_previous*G_t.' + V_t*M_t*(V_t');

%% Measurement update (correction step) 
Q_t = [0.01^2 0 0
       0 0.1^2 0
       0 0 0];

% loop to iterate through all observed fiducials
for i=1:12
    % this if statement skips the loop iteration when no fiducial is observed
    % this is because z_t is a 12x3 matrix, one row for each landmark in the map
    if z_t(i,1) == 0
        continue;
    end
    
    % extract correspondence of observed fiducial
    j = c_t(i,1);
    
    % calculate difference between map coords of fiducial and the pose of the robot
    q = (m(j,1) - mew_hat_t(1))^2 + (m(j,2) - mew_hat_t(2))^2;
       
    % z_hat_t is the predicted measurement
    z_hat_t = [sqrt(q)
               (atan2((m(j,2) - mew_hat_t(2)), (m(j,1) - mew_hat_t(1))) - mew_hat_t(3))
               0];
      
    % H_t is the jacobian of the measurement model
    % the last row of H_t are all 0 because the correct correspondence of each fiducial is known
    H_t = [-((m(j,1) - mew_hat_t(1))/sqrt(q)) -((m(j,2) - mew_hat_t(2))/sqrt(q)) 0
           ((m(j,2) - mew_hat_t(2))/q) -((m(j,1) - mew_hat_t(1))/q) -1
           0                                        0                            0];
    
    % S_t is the overall measurement prediction uncertainty
    % the robot location uncertainty is mapped into observation uncertainty by multiplying by H_t
    S_t = H_t*sigma_hat_t*H_t.' + Q_t;
    
    % compute the inverse of the matrix S_t
    % pinv is used to compute the pseudoinverse as the determinant is often 0 and so inverse does not exist
    S_t_inverse = pinv(S_t);
    
    % K_t is the kalman gain matrix
    % the offset between the predicted and observed measurement is mapped into state space
    % used to move the location estimate in the direction that would reduce the measurement innovation
    % the more certain the observation the higher the kalman gain
    K_t = sigma_hat_t*H_t.'*S_t_inverse;

    % mew_hat_t is the updated position estimate
    % the kalman gain K_t is used to map the difference between predicted and observed measurement into state space
    mew_hat_t = mew_hat_t + (K_t*(z_t(i,:).' - z_hat_t)).';
    
    % sigma_hat_t is the uncertainty elipse of the location estimate 
    sigma_hat_t = (I - K_t*H_t)*sigma_hat_t;
end

% mew_t is updated with the best estimate for output
mew_t = mew_hat_t;

% sigma_t is updated with the best estimate for output
sigma_t = sigma_hat_t;

% p_zt = det((2*pi*S_t)^-0.5) * exp(-0.5*(z_t - z_hat_t).'*(S_t^-1)*(z_t - z_hat_t));
p_zt=0; % value is not necessary for filter to work, included for completeness of algorithm
end








