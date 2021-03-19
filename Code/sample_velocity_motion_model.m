%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Algorithm sample motion model velocity, P.124 Thrun
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Martin J. Pearson, Probabilistic Robotics, UWE
%%%%%
function [x] =  sample_velocity_motion_model(u, x_1, delta_t)
%%% Arguments:
% u is a 1x2 vector of most recent motor command velocity (linear angular)
% x_1 is the previous pose estimate [x y theta]
% delta_t is the update period
%%% Return:
% x is the current pose estimate [x y theta]

alpha1 = 0.1;       % noise measured in rotation caused by rotation
alpha2 = 0.02;      % noise measured in rotation caused by translation
alpha3 = 0.2;       % noise measured in translation caused by translation
alpha4 = 0.01;      % noise measured in translation caused by rotation

alpha5 = 0.001;     % additional noise measured in rotation caused by translation
alpha6 = 0.01;      % additional noise measured in rotation caused by rotation

x       = zeros(3,1);       % initialise output vector
v       = u(1);             % extract linear velocity
w       = u(2);             % extract angular velocity
theta   = x_1(3);           % extract direction of robot

% Add noise taken from distribution that represents p(x|u,x_1)
v_hat       = v  + sample_norm((alpha1*v^2) + (alpha2*w^2));
w_hat       = w  + sample_norm((alpha3*v^2) + (alpha4*w^2));
gamma_hat   = sample_norm((alpha5*v^2) + (alpha6*w^2));

if(w == 0)
    % forward model of motion to find sample estimate of pose
    x(1) = x_1(1) + v_hat*delta_t*cos(theta);
    x(2) = x_1(2) + v_hat*delta_t*sin(theta);
    x(3) = wrapToPi(theta + gamma_hat*delta_t);
else
    % forward model of motion to find sample estimate of pose
    x(1) = x_1(1) - (v_hat/w_hat)*sin(theta) + (v_hat/w_hat)*sin(theta+w_hat*delta_t);
    x(2) = x_1(2) + (v_hat/w_hat)*cos(theta) - (v_hat/w_hat)*cos(theta+w_hat*delta_t);
    x(3) = wrapToPi(theta + w_hat*delta_t + gamma_hat*delta_t);
end

% Sample from a Gaussian (p.124 Thrun)
function [x] = sample_norm(b)

b=sqrt(b);

x = 0.5* sum((rand(1,12)*(b*2))-b);
