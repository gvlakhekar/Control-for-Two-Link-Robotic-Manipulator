%--------------------------------------------------------------------------
% Neural Network Control for Two Link Robotic Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%--------------------------------------------------------------------------
% Define the neural network architecture
hiddenLayerSize = 10; % Number of neurons in the hidden layer 
net = feedforwardnet(hiddenLayerSize);

% Specify the training function and options
net.trainFcn = 'trainlm'; % Levenberg-Marquardt backpropagation 
net.layers{1}.transferFcn = 'tansig'; % Transfer function for the hidden layer 
net.layers{2}.transferFcn = 'purelin'; % Linear transfer function for output layer

% Set training parameters (e.g., max epochs, error tolerance) 
net.trainParam.max_fail = 10; % Maximum validation failures before stopping 
net.trainParam.epochs = 1000; % Max epochs for training 
net.trainParam.min_grad = 1e-6; % Minimum gradient for convergence

% Simulation parameters
n_samples = 1000; % Number of data samples
q_range = [-pi, pi]; % Range for joint angles (q1, q2) 
dq_range = [-2, 2];	% Range for joint velocities (dq1, dq2)
q_samples = linspace(q_range(1), q_range(2), n_samples); % Sample joint angles 
dq_samples = linspace(dq_range(1), dq_range(2), n_samples); % Sample joint velocities

% Pre-allocate data arrays
inputs = zeros(n_samples^2, 4); % [q1, q2, dq1, dq2] 
outputs = zeros(n_samples^2, 2); % [tau1, tau2]
% Generate data 
index = 1;
for i = 1:n_samples 
    for j = 1:n_samples
q = [q_samples(i), q_samples(j)]; % Joint angles
dq = [dq_samples(i), dq_samples(j)]; % Joint velocities
% Get the dynamics matrices
[M, C, G] = robotDynamics(q, dq);
% Assume desired accelerations (ddq) (this could come from a control strategy) 
 ddq = [0; 0]; % For simplicity, we set desired accelerations to zero
% Compute the torques (tau = M*ddq + C*dq + G) 
tau = M * ddq + C * dq' + G;

% Store the input and output data
inputs(index, :) = [q, dq]; % q1, q2, dq1, dq2 as inputs 
outputs(index, :) = tau';	% tau1, tau2 as outputs 
index = index + 1; 

    end

end
 
% Train the neural network
[net, tr] = train(net, inputs', outputs');

% Save the trained network 
save('trained_network.mat', 'net');

% Test the trained network with new data
 
q_test = [pi/4, pi/6];	% Test joint angles (q1, q2) 
dq_test = [1, 0.5];		% Test joint velocities (dq1, dq2)
% Prepare the input vector 
input_test = [q_test, dq_test];

% Predict the torques using the trained neural network 
predicted_torques = net(input_test');

% Display the predicted torques 
disp('Predicted torques:'); 
disp(predicted_torques);
% Load trained neural network controller 
load('NeuralNetworkModel.mat'); % assumes 'net' is the network variable

%% Robot parameters 
m1 = 1.0; m2 = 1.0;
l1 = 1.0; l2 = 1.0;
g = 9.81;
dt = 0.01;
t = 0:dt:10; N = length(t);
%% Closed-Loop Simulation q = [0; 0];
q_dot = [0; 0]; 
Q_history = zeros(2, N); 
U_history = zeros(2, N);
q_traj = [1;0.5]*ones([1 N]); 
qd_dot_traj = [0;0]*ones([1 N]);
for i = 1:N
q_des = q_traj(:, i); 
qd_dot = qd_dot_traj(:, i);

input_vector = [q' q_dot' q_des' qd_dot']; 
u = predict(net, input_vector);

M = [m1*l1^2 + m2*(l1^2 + 2*l1*l2*cos(q(2)) + l2^2), m2*(l1*l2*cos(q(2)) + l2^2); m2*(l1*l2*cos(q(2)) + l2^2),	m2*l2^2];
C = [-m2*l1*l2*sin(q(2))*q_dot(2), -m2*l1*l2*sin(q(2))*(q_dot(1)+q_dot(2)); m2*l1*l2*sin(q(2))*q_dot(1), 0];

G = [ (m1 + m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1) + q(2)); m2*g*l2*cos(q(1) + q(2)) ];

q_ddot = M \ (u' - C * q_dot - G); 
q_dot = q_dot + q_ddot * dt;
q = q + q_dot * dt;

Q_history(:, i) = q; 
U_history(:, i) = u';
 
end
%% Plot Results
% Plot figure;
plot(t, Q_history(1,:), 'r', t, Q_history(2,:), 'b'); hold on;
plot(t, 0.7*q_traj(1,:), 'r--', t, 0.5*q_traj(2,:), 'b--'); xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Step Response with Neural Network Control'); legend('q1', 'q2', 'q1 desired', 'q2 desired'); grid on;

