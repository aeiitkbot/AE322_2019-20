function [ M ] = Control(X, Xd)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
        
% Gains
kp = 10;
kd = 1;

    
% Actual and desired states
att  = X(1:3,:); 
omega  = X(4:6,:);
att_d = Xd(1:3,:);
omega_d = Xd(4:6,:);
  
% Error Euler angle
att_e = [att - att_d];
   
% Omega error
omega_e = omega-omega_d;
    
% Control 
% Moment desired calculated using quaternion error
M = -kp*att_e-kd*omega_e;
        
        


end

