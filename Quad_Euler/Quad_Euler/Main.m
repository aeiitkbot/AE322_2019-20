% This program is written by Dr. Kothari
% The purpose to compare quaternion based control algorithms

close all;
clear all;
clc;

global dt

% Initlization 

dt     = 0.01;
N      = 3000;

% Memory allocation 

% PD controller state
X = zeros(6,N);
U = zeros(3, N-1);
Euler_des = zeros(3, N-1);
omega_ref = zeros(3,N-1);

% Initial states
t(:,1) = 0;
att   = [0;10*(pi/180);0];
omega  = [0;0;0];
X(:,1) = [att
          omega ];
% Control gains
kp = 10;
kd = 0.1;
%Moment of Inertia
Iyy = 0.25;
%Closed loop transfer function
Gcl = tf([kd/Iyy, kp/Iyy], [1, kd/Iyy, kp/Iyy]);

fb = bandwidth(Gcl);
w = fb/2; % maximum frequency
T1 = 2*pi/w; %minimum time period
fprintf('Maximum frequency that can be tracked:%f\n',w);
fprintf('Corresponding time-period:%f\n',T1);
figure(1)
bode(Gcl);
% Time period
T = 2; % sec

for k = 1:N-1
    
   

    % Desired attitude trajectory in terms of Euler angles
    Xd(:,k) = [0; 0; 0; 0; 0; 0];
    
    Xd(2,k) = 0.1*sin((2*pi/T)*t(:,k)); 
    
        
    % Control computation 
    U(:,k)  = Control(X(:,k), Xd(:,k), kp, kd);
    
        
    % State update
        
    X(:,k+1) = X(:,k) + dt*rot_kin_dyn(X(:,k), U(:,k));
    
     % angle adjustment 
    X(1,k+1) = ang_wrap(X(1,k+1));
    X(2,k+1) = ang_wrap(X(2,k+1));
    X(3,k+1) = ang_wrap(X(3,k+1));
    
    % Time update
    
    t(:,k+1) = t(:,k)+dt;
    
        
end



% Plotting 

% Quaternion Attitude
figure(2)
subplot(3,1,1)
plot(t,X(1,:)*(180/pi),'k-',t(:,1:N-1),Xd(1,:)*(180/pi));
ylabel('\phi^0');
subplot(3,1,2)
plot(t,X(2,:)*(180/pi),'k-',t(:,1:N-1),Xd(2,:)*(180/pi));
ylabel('\theta^0');
subplot(3,1,3)
plot(t,X(3,:)*(180/pi),'k-',t(:,1:N-1),Xd(3,:)*(180/pi));
% legend('q0', 'q1', 'q2', 'q3', 'q0_{ref}', 'q1_{ref}', 'q2_{ref}', 'q3_{ref}')
xlabel('Time (sec)');
ylabel('\psi^0');

% Angular velocity 
figure(3)
subplot(3,1,1)
plot(t,X(4,:));
ylabel('p');
subplot(3,1,2)
plot(t,X(5,:));
ylabel('q');
subplot(3,1,3)
plot(t,X(6,:));
ylabel('r');
xlabel('Time (sec)');

figure(4)
subplot(3,1,1)
plot(t(:,1:N-1),U(1,:),'k--');
ylabel('l');
subplot(3,1,2)
plot(t(:,1:N-1),U(2,:),'k--');
ylabel('m');
subplot(3,1,3)
plot(t(:,1:N-1),U(3,:),'k--');
ylabel('n');
xlabel('Time (sec)');

