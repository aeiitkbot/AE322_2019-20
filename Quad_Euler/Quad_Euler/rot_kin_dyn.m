function XX_dot = rot_kin_dyn(XX, U)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% System parameters
J = 10*diag([0.025, 0.025, 0.5]);
% states 
phi  = XX(1,:);
theta = XX(2,:);
omega = XX(4:6,:);


% Kinematics
att_dot= [1   sin(phi)*tan(theta)  cos(phi)*tan(theta)
          0   cos(phi)            -sin(phi)
          0   sin(phi)*sec(theta) cos(phi)*sec(theta)]*[omega(1,1); omega(2,1); omega(3,1)];

    
% Attitude dynamics 
        
omega_hat=[ 0 -omega(3,1) omega(2,1);
            omega(3,1) 0 -omega(1,1);
            -omega(2,1) omega(1,1) 0];
    
omega_dot=inv(J)*(-omega_hat*(J*omega)+U);
        

XX_dot = [att_dot
          omega_dot ];
                    
    
end

