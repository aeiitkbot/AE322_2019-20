% Control design for quadcopter 

clear all; clc;
% Transfer function for pitch dynamics 
% Double integrator system 
Iyy = 0.25; % Moment of inertia along y-axis kg-m^2
G = tf(1, [Iyy*1, 0, 0]);
% bode plot 
figure(1)
bode(G);

% PD controller 
% Transfer function of PD controller
% kp + kd s
% closed loop transfer function 
kp = 10;
kd = 0.7;
Gcl = tf([kd/Iyy, kp/Iyy], [1, kd/Iyy, kp/Iyy]);
figure(2)
bode(Gcl)

% figure(3)
% bode(G, 'r', Gcl, 'b');