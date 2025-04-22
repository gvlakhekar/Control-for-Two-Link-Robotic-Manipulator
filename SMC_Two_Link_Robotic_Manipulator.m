%--------------------------------------------------------------------------
% Sliding Mode Control for Two Link Robotic Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%--------------------------------------------------------------------------
% Simulation setup 
dt = 0.001;
T = 5;
time = 0:dt:T;
n = length(time);

% Desired joint trajectory (step)
q_d = [-pi/2; pi/6]; 
dq_d = [0; 0];
ddq_d = [0; 0];

% Initial states 
q = [0; 0];
dq = [0; 0];

% Logs
q_log = zeros(2, n); 
q_dot_log = zeros(2, n);

% SMC gains
lambda = diag([10, 10]);
eta = [5; 5]; % switching gain

% Simulation loop for SMC 

for i = 1:n

% Errors
e = q - q_d;
de = dq - dq_d;
s = lambda * e + de;

% Get dynamics
[M, C, G] = robotDynamics(q, dq);

% Control law
tau = -lambda * de - C*dq - G - eta .* sign(s);

% Forward dynamics
ddq = M \ (tau - C*dq - G);

% Integrate
dq = dq + ddq * dt; q = q + dq * dt;
% Log
q_log(:, i) = q; 
q_dot_log(:, i) = dq;

end
 
% Plot SMC results figure;
plot(time, q_log(1,:), 'r', time, q_log(2,:), 'b'); hold on;
yline(q_d(1), 'r--'); yline(q_d(2), 'b--'); xlabel('Time (s)'); ylabel('Joint Angles (rad)'); title('Joint Space Control of 2-R Manipulator Using Sliding Mode Control'); legend('q1', 'q2', 'q1 Desired', 'q2 Desired'); grid on;
