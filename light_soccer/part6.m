clear all;clc;close all;
load('reference_input_state.mat');
load('trajectory_gk.mat');
%% simulate with N
N=12;
xmax=16.5+(7.32/2);
ymax=16.5;
%
gainF=1;

figureName='MPC_N';
[x_N,u_N,calc_time_N] = part6_simulate(N,figureName,xmax,ymax,gainF);

figureName='MPC_2_N';
[x_2_N,u_2_N,calc_time_2_N] = part6_simulate(2*N,figureName,xmax,ymax,gainF);

figureName='MPC_0_5_N';
[x_0_5_N,u_0_5_N,calc_time_0_5_N] = part6_simulate(0.5*N,figureName,xmax,ymax,gainF);
%%
table=[...
    N/2,N,2*N; ...
    mean(calc_time_0_5_N),mean(calc_time_N),mean(calc_time_2_N)...
    ];

rowLabels = {'N', 't(s)'};
matrix2latex(table, './report/tables/MPC_state_const_time_table.tex', ...
    'rowLabels', rowLabels, ...
    'alignment', 'c', 'format', '%-6.2e', 'size', 'tiny');

J_N = total_sim_cost_MPC(x_N,u_N);
J_0_5_N = total_sim_cost_MPC(x_0_5_N,u_0_5_N);
J_2_N = total_sim_cost_MPC(x_2_N,u_2_N);

table=[...
    N/2,N,2*N; ...
    J_0_5_N,J_N, J_2_N; ...
    0 (J_N-J_0_5_N)/J_N (J_2_N-J_N)/J_2_N...
    ];

rowLabels = {'horizon', 'J','relative difference previous value of J'};
matrix2latex(table, './report/tables/MPC_state_const_cost_table.tex', ...
    'rowLabels', rowLabels, ...
    'alignment', 'c', 'format', '%-6.2e', 'size', 'tiny')
%%
fig=figure;
plot(x_N(1,:),x_N(2,:),'.');hold all;
plot(x_2_N(1,:),x_2_N(2,:),'.');hold all;
plot(x_0_5_N(1,:),x_0_5_N(2,:),'.');hold all;
plot(x_ref(1,:),x_ref(2,:),'O'); hold all;
plot(y(1,:),y(2,:),'black'); hold all;
legend('N','2N','0.5N','reference');
xlabel('x');ylabel('y');
saveas(fig,'./report/img/MPC_state_constraint/compare.png');

fig=figure;
plot(x_N(1,:),x_N(2,:),'.');hold all;
plot(x_2_N(1,:),x_2_N(2,:),'.');hold all;
plot(x_0_5_N(1,:),x_0_5_N(2,:),'.');hold all;
plot(x_ref(1,:),x_ref(2,:),'O'); hold all;
plot(y(1,:),y(2,:),'black'); hold all;
title('zoomed in on active constraint');
xlabel('x');ylabel('y');
legend('N','2N','0.5N','reference');
xlabel('x');ylabel('y');
xlim([-11 -3]);ylim([13 17]);
saveas(fig,'./report/img/MPC_state_constraint/compare_zoom.png');
%%
gainF=10;

figureName='MPC_N_F10';
[x_N,u_N,calc_time_N] = part6_simulate(N,figureName,xmax,ymax,gainF);

figureName='MPC_2_N_F10';
[x_2_N,u_2_N,calc_time_2_N] = part6_simulate(2*N,figureName,xmax,ymax,gainF);

figureName='MPC_0_5_N_F10';
[x_0_5_N,u_0_5_N,calc_time_0_5_N] = part6_simulate(0.5*N,figureName,xmax,ymax,gainF);
%%
fig=figure;
plot(x_N(1,:),x_N(2,:),'.');hold all;
plot(x_2_N(1,:),x_2_N(2,:),'.');hold all;
plot(x_0_5_N(1,:),x_0_5_N(2,:),'.');hold all;
plot(x_ref(1,:),x_ref(2,:),'O'); hold all;
plot(y(1,:),y(2,:),'black'); hold all;

xlabel('x');ylabel('y');
legend('N','2N','0.5N','reference');
saveas(fig,'./report/img/MPC_state_constraint/compare_F10.png');

fig=figure;
plot(x_N(1,:),x_N(2,:),'.');hold all;
plot(x_2_N(1,:),x_2_N(2,:),'.');hold all;
plot(x_0_5_N(1,:),x_0_5_N(2,:),'.');hold all;
plot(x_ref(1,:),x_ref(2,:),'O'); hold all;
plot(y(1,:),y(2,:),'black'); hold all;
title('zoomed in on active constraint');
xlabel('x');ylabel('y');
legend('N','2N','0.5N','reference');
xlim([-11 -3]);ylim([13 17]);
saveas(fig,'./report/img/MPC_state_constraint/compare_F10_zoom.png');
%%
gainF=1;
figureName='inf';
[x,u,calc_time] = part6_simulate(5,figureName,xmax,ymax,gainF);
gainF=10;
figureName='inf_F10';
[x,u,calc_time] = part6_simulate(5,figureName,xmax,ymax,gainF);