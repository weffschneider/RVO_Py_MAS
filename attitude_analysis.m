% AA 277, Winter 2019
% 3/12/19
%
% Project
%
clear
close all
%% 
dt = 0.01;
s = tf('s');

G = 1/s; % single integrator dynamics
K = 1;

sys_cont = feedback(G*K,1);
sys_dis = c2d(sys_cont, dt);

figure
subplot(211)
step(sys_dis)
grid on
title('Output Step Response: \Deltat = 0.01s')
ylabel('\theta (rad)')

u_cont = feedback(K, G); % reference to omega
u_dis = c2d(u_cont, dt);
subplot(212)
step(u_dis)
grid on
title('Control Input Step Response: \Deltat = 0.01s')
ylabel('\omega (rad/s)')