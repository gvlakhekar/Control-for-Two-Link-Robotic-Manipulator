%--------------------------------------------------------------------------
% Backstepping Control for Two Link Robotic Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%--------------------------------------------------------------------------
clc; 
close all;
%% Robot Parameters
m1 = 1; m2 = 1;	% masses
l1 = 1; l2 = 1;	% link lengths 
lc1 = l1/2; lc2 = l2/2; % COM positions 
I1 = 0.1; I2 = 0.1;	% inertias
g = 9.81;
%% Time Settings 
dt = 0.001;
T = 5;
t = 0:dt:T;
%% Desired Trajectory
qd = [ones([1 length(t)]); 1.5*ones([1 length(t)])]; qd_dot = zeros([2 length(t)]);
qd_ddot = zeros([2 length(t)]);
%% Initialize States
q = [0.5; 0.1];	% initial joint positions 
q_dot = [0; 0];	% initial joint velocities
Q = q; QD = q_dot; TAU = [];

%% Backstepping Gains
K1 = diag([2, 2]);	% damping gain 
K2 = diag([5, 5]);	% stiffness gain

for i = 1:length(t)-1
% Current state
q1 = q(1); q2 = q(2);
dq1 = q_dot(1); dq2 = q_dot(2);

% Desired trajectory 
qd_i = qd(:,i); qd_dot_i = qd_dot(:,i);
qd_ddot_i = qd_ddot(:,i);
% Errors
z1 = q - qd_i;
z2 = q_dot - qd_dot_i;
%% Robot Dynamics
% Inertia matrix M(q)
M11 = I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q2); M12 = I2 + m2*l1*lc2*cos(q2);
M21 = M12; M22 = I2;
M = [M11, M12; M21, M22];

% Coriolis and centrifugal matrix C(q, dq) 
h = -m2*l1*lc2*sin(q2);
C = [h*dq2, h*(dq1 + dq2);-h*dq1, 0];

% Gravity vector G(q)
G1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2); G2 = m2*lc2*g*cos(q1 + q2);
G = [G1; G2];

%% Backstepping Control Law
v = qd_ddot_i - K1*z2 - K2*z1; tau = M*v + C*q_dot + G;

%% Euler Integration
q_ddot = M \ (tau - C*q_dot - G); q_dot = q_dot + q_ddot * dt;
q = q + q_dot * dt;
% Store 
Q(:,i+1) = q;
QD(:,i+1) = q_dot;
TAU(:,i) = tau;

end
 
%% Plots 
figure;
plot(t, Q(1,1:length(t)), 'r', t, qd(1,:), 'r--')
hold on
plot(t, Q(2,1:length(t)), 'b', t, qd(2,:), 'b--') 
title('Joint Space Control of 2R Manipulator Using Backstepping Method')
xlabel('Time (s)')
ylabel('Joint Angles (rad)')
legend('q1', 'q1 Desired', 'q2', 'q2 Desired')
