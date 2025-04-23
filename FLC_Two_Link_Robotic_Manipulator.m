
% Create Sugeno-type FIS
fis = sugfis('Name', 'TunedFLC');
% Input: error (e)
fis = addInput(fis, [-pi pi], 'Name', 'e');
fis = addMF(fis, 'e', 'gaussmf', [0.7 -pi], 'Name', 'N');
fis = addMF(fis, 'e', 'gaussmf', [0.7 0],	'Name', 'Z');
fis = addMF(fis, 'e', 'gaussmf', [0.7 pi], 'Name', 'P');

% Input: derivative of error (de)
fis = addInput(fis, [-2*pi 2*pi], 'Name', 'de');
fis = addMF(fis, 'de', 'gaussmf', [1.5 -2*pi], 'Name', 'N');
fis = addMF(fis, 'de', 'gaussmf', [1.5 0],	'Name', 'Z');
fis = addMF(fis, 'de', 'gaussmf', [1.5 2*pi], 'Name', 'P');

% Output: torque (tau)
fis = addOutput(fis, [-10 10], 'Name', 'tau'); 
for i = 1:9
fis = addMF(fis, 'tau', 'constant', 0, 'Name', ['out' num2str(i)]);
end

% Tuned linear output params: [e_coeff, de_coeff, offset] 
outputParams = [
	-2.0	-1.5	0;
	-1.5	-1.0	0;
	-1.0	-0.5	0;
	0.0	-0.5	0;
	0.0	0.0	0;
	0.0	0.5	0;
	1.0	0.5	0;
	1.5	1.0	0;
	2.0	1.5	0
];			

% Add rules and assign output functions 
ruleList = [];
k = 1;
for i = 1:3
for j = 1:3
ruleList(end+1,:) = [i j k 1 1]; % last two: weight=1, AND 
fis.Output.MembershipFunctions(k).Type = 'linear';
fis.Output.MembershipFunctions(k).Parameters = outputParams(k,:); 
k = k + 1;
end 
end
 
fis = addRule(fis, ruleList);

% Copy for joint 2 
fis2 = fis; 
fis2.Name = 'FLC2';
dt = 0.01;
T = 15;
time = 0:dt:T;
n = length(time);

% Desired positions 
q_d = [2*pi/3; -pi/6]; 
dq_d = [0; 0];
ddq_d = [0; 0];

% Initial states 
q = [0; 0];
dq = [0; 0];
% Logs
q_log = zeros(2, n); 
dq_log = zeros(2, n);

for i = 1:n
% Error and derivative 
e = q - q_d;
de = dq - dq_d; 
q = wrapToPi(q);

% FLC control torques
tau1 = evalfis(fis, [e(1), de(1)]);
tau2 = evalfis(fis2, [e(2), de(2)]); 
tau = [tau1; tau2];

% Dynamics
[M, C, G] = robotDynamics1(q, dq); 
ddq = M \ (tau - C*dq - G);

% Integration
dq = dq + ddq * dt; 
q = q + dq * dt;

% Logging 
q_log(:, i) = q;
dq_log(:, i) = dq;
  
end
 
% Plot
figure;
plot(time, q_log(1,:), 'r', time, q_log(2,:), 'b'); hold on;
yline(-q_d(1)/2, 'r--'); yline(q_d(2), 'b--'); xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Joint Space Control of 2R Manipulator Using FLC'); legend('q1', 'q2', 'q1 Desired', 'q2 Desired'); grid on;
