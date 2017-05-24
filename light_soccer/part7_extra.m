clear all;clc;close all;
load('reference_input_state.mat');
load('trajectory_gk.mat');
N=12;

region_of_interest_x = [1 10];
region_of_interest_y = [3 11];

%% with term cost
figureName='NOSAVE';
[x_N,u_N,calc_time_N] = part7_simulate(N,figureName);
[x_0_5_N,u_0_5_N,calc_time_0_5_N] = part7_simulate(0.5*N,figureName);
[x_2_N,u_2_N,calc_time_2_N] = part7_simulate(2*N,figureName);
%%
table=[...
    N/2,N,2*N; ...
    mean(calc_time_0_5_N),mean(calc_time_N),mean(calc_time_2_N)...
    ];

rowLabels = {'N', 't(s)'};
matrix2latex(table, './report/tables/MPC_term_cost_time_table.tex', ...
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
matrix2latex(table, './report/tables/MPC_term_cost_cost_table.tex', ...
    'rowLabels', rowLabels, ...
    'alignment', 'c', 'format', '%-8.4e', 'size', 'tiny');
close all;
%% without term cost
figureName='NOSAVE';
[x_N_old,u_N_old,calc_time_N_old] = part5_simulate(N,figureName);
[x_0_5_N_old,u_0_5_N_old,calc_time_0_5_N_old] = part5_simulate(N/2,figureName);
[x_2_N_old,u_2_N_old,calc_time_2_N_old] = part5_simulate(2*N,figureName);
close all;
%%
fig=figure;
plot(x_N(1,:),x_N(2,:),'Color','blue');hold all;
plot(x_N_old(1,:),x_N_old(2,:),'.','Color','blue');hold all;

plot(x_0_5_N(1,:),x_0_5_N(2,:),'Color','green');hold all;
plot(x_0_5_N_old(1,:),x_0_5_N_old(2,:),'.','Color','green');hold all;

plot(x_2_N(1,:),x_2_N(2,:),'Color','red');hold all;
plot(x_2_N_old(1,:),x_2_N_old(2,:),'.','Color','red');hold all;

% plot(x_ref(1,:),x_ref(2,:),'+','Color','black');
% plot(y(1,:),y(2,:),'Color','black');
legend(...
    'with terminal cost horizon=N','without terminal cost horizon=N' ...
    ,'with terminal cost horizon=N/2','without terminal cost horizon=N/2' ...
    ,'with terminal cost horizon=2N','without terminal cost horizon=2N' ...
    ,'Location', 'southeast' ...
    );
%     ,'reference path','theoretical path' 
xlim(region_of_interest_x);
ylim(region_of_interest_y);
xlabel('x');ylabel('y');
saveas(fig,'./report/img/MPC_term_cost/compare_ROI_all.png');