%% PENCE MATARIA 
%  AE322 (170382)  

clear all;

%% INPUTS
prompt = {'Enter Damping Ratio:','Enter Natural Frequency:','Enter Step size:'};
dlgtitle = 'Input';
dims = [1 35];
definput = {'0.44','1.8','0.05'};
answer = inputdlg(prompt,dlgtitle,dims,definput);
si = str2double(answer(1));
wn = str2double(answer(2));
h = str2double(answer(3));

%%
t(1)=0;
tfinal = 20;
N=ceil(tfinal/h);
y(1) = -1;
y_dot(1) = 1;

F_y = @(t,y,y_dot) y_dot;                       %Dynamical Diff. equations for second order system
F_y_dot = @(t,y,y_dot) -(wn^2)*y-2*si*wn*y_dot;

%% RK4 LOOP
for i=1:(N-1)
    t(i+1) = t(i) + h;
    k_1_y = F_y(t(i),y(i),y_dot(i));
    k_1_y_dot = F_y_dot(t(i),y(i),y_dot(i));
    k_2_y = F_y(t(i)+0.5*h,y(i)+0.5*h*k_1_y,y_dot(i)+0.5*h*k_1_y_dot);
    k_2_y_dot = F_y_dot(t(i)+0.5*h,y(i)+0.5*h*k_1_y,y_dot(i)+0.5*h*k_1_y_dot);
    k_3_y = F_y(t(i)+0.5*h,y(i)+0.5*h*k_2_y,y_dot(i)+0.5*h*k_2_y_dot);
    k_3_y_dot = F_y_dot(t(i)+0.5*h,y(i)+0.5*h*k_2_y,y_dot(i)+0.5*h*k_2_y_dot);
    k_4_y = F_y(t(i)+h,y(i)+h*k_3_y,y_dot(i)+h*k_3_y_dot);
    k_4_y_dot = F_y_dot(t(i)+h,y(i)+h*k_3_y,y_dot(i)+h*k_3_y_dot);
    
    y(i+1) = y(i) + (1/6)*(k_1_y + 2*k_2_y + 2*k_3_y + k_4_y)*h; 
    y_dot(i+1) = y_dot(i) + (1/6)*(k_1_y_dot + 2*k_2_y_dot + 2*k_3_y_dot + k_4_y_dot)*h;
end

%% PLOTTING
subplot(1,2,1);
plot(t,y,'-');
xlabel('t');
ylabel('y');
subplot(1,2,2);
plot(t,y_dot,'-');
xlabel('t');
ylabel('y dot')