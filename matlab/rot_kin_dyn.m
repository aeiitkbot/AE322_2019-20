
function XX_dot = rot_kin_dyn(t, XX)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% System parameters
J = 10*diag([0.025, 0.025, 0.5]);
% states
quat  = XX(1:4,:);
omega = XX(5:7,:);
M     = XX(8:10,:);

% Kinematics
quat_dot=0.5*quatmultiply(quat,[0;omega]);


% Attitude dynamics

omega_hat=[ 0 -omega(3,1) omega(2,1);
            omega(3,1) 0 -omega(1,1);
            -omega(2,1) omega(1,1) 0];

omega_dot=inv(J)*(-omega_hat*(J*omega)+M);

% Actuator dynamics
M_dot = zeros(3,1);


XX_dot = [quat_dot
          omega_dot
          M_dot ];


end


