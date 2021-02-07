% This program is written by Dr. Kothari
% The purpose to compare quaternion based control algorithms

close all
clear all
clc

global dt

% Initlization 

dt     = 0.01;
N      = 1000;

% Memory allocation 

% PD controller state
X = zeros(6,N);
U = zeros(3, N-1);
Euler = zeros(3, N-1);
Euler_des = zeros(3, N-1);
omega_ref = zeros(3,N-1);

% Initial states
t(:,1) = 0;
quat   = [0;0;0];
omega  = [0;0;0];
X(:,1) = [quat
          omega ];


for k = 1:N-1
    
   

    % Desired attitude trajectory in terms of Euler angles
    % Xd(:,k) = desired_traj(t(:,k));
    
        
    % Control computation 
    U(:,k)  = [0; 0; 0]; %quad_attitude_control_PD(X(:,k), Xd(:,k));
    
        
    % State update
    
    X(:,k+1) = X(:,k) + dt*rot_kin_dyn(X(:,k), U(:,k));
    % Time update
    
    t(:,k+1) = t(:,k)+dt;
    
        
end



% Plotting 

% Quaternion Attitude
figure(1)
plot(t,X(1,:),'k-',t,X(2,:),'k--',t,X(3,:),'k.-',t);
% legend('q0', 'q1', 'q2', 'q3', 'q0_{ref}', 'q1_{ref}', 'q2_{ref}', 'q3_{ref}')

% Angular velocity 
figure(2)
plot(t,X(4,:),'k-',t,X(5,:),'k--',t,X(6,:),'k.-');

figure(3)
subplot(3,1,1)
plot(t(:,1:N-1),U(1,:),'k--');
subplot(3,1,2)
plot(t(:,1:N-1),U(2,:),'k--');
subplot(3,1,3)
plot(t(:,1:N-1),U(3,:),'k--');
