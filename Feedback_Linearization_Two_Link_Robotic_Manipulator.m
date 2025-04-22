%--------------------------------------------------------------------------
% Feedback Linearization Control for Two Link Robotic Manipulator  
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
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
% Reset states
q = [0; 0]; 
dq = [0; 0];
q_log = zeros(2, n); 
q_dot_log = zeros(2, n);
% PD gains
Kp = diag([100, 100]);
Kd = diag([20, 20]);

for i = 1:n
% Errors
e = q - q_d;
de = dq - dq_d;

% Desired joint acceleration 
v = ddq_d - Kd*de - Kp*e;

% Get dynamics
[M, C, G] = robotDynamics(q, dq);

% Feedback linearization control law 
tau = M*v + C*dq + G;
% Forward dynamics
ddq = M \ (tau - C*dq - G);
% Integrate
dq = dq + ddq * dt; q = q + dq * dt;
q = q + dq * dt;
% Log
q_log(:, i) = q;
q_dot_log(:, i) = dq;
end

% Plot
plot(time, q_log(1,:), 'r', time, q_log(2,:), 'b'); hold on;
yline(q_d(1), 'r--'); yline(q_d(2), 'b--'); xlabel('Time (s)'); ylabel('Joint Angles (rad)'); title('Joint Space Control Using Feedback Linearization'); legend('q1', 'q2', 'q1 Desired', 'q2 Desired'); grid on;
