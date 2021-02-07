%% PENCE MATARIA 
%  AE322 (170382)  

clear all;

%% INPUTS
prompt = {'Enter Time Constant:','Enter step size:'};
dlgtitle = 'Input';
dims = [1 35];
definput = {'1','0.05'};
answer = inputdlg(prompt,dlgtitle,dims,definput);
T = str2double(answer(1));
h= str2double(answer(2));

%% 
t(1)=0;                %initializing time
tfinal = 20;
N=ceil(tfinal/h);      %Time Interval
y(1) = 0;
F_y = @(t,y,y_dot) -(y/T)+1;     % Dynamical Diff. Function for First order system

%% RK4 LOOP
for i=1:(N-1)
    t(i+1) = t(i) + h;
    k_1_y = F_y(t(i),y(i));
    k_2_y = F_y(t(i)+0.5*h,y(i)+0.5*h*k_1_y);
    k_3_y = F_y(t(i)+0.5*h,y(i)+0.5*h*k_2_y);
    k_4_y = F_y(t(i)+h,y(i)+h*k_3_y);
    
    y(i+1) = y(i) + (1/6)*(k_1_y + 2*k_2_y + 2*k_3_y + k_4_y)*h; 
end

%% PLOTTING
plot(t,y,'-');
xlabel('t');
ylabel('y');