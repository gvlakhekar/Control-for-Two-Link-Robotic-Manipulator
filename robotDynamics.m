%--------------------------------------------------------------------------
% Dynamic Modeling of Two Link Robotic Manipulator  
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% Dynamics function
function [M, C, G] = robotDynamics(q, dq)

m1 = 1; m2 = 1; l1 = 1; l2 = 1; lc1 = 0.5; lc2 = 0.5;
I1 = 0.1; I2 = 0.1; g = 9.81;

a = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2);
b = m2*l1*lc2;
d = I2 + m2*lc2^2;

M = [a + 2*b*cos(q(2)), d + b*cos(q(2));
 
d + b*cos(q(2)),	d];
C = [-b*sin(q(2))*dq(2), -b*sin(q(2))*(dq(1)+dq(2)); b*sin(q(2))*dq(1), 0];
G = [(m1*lc1 + m2*l1)*g*cos(q(1)) + m2*lc2*g*cos(q(1)+q(2)); m2*lc2*g*cos(q(1)+q(2))];

end
 