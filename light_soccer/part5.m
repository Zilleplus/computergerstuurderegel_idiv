clear all;clc;close all;
N=12;
%% simulate with N
figureName='MPC_N';
[x_N,u_N,calc_time_N] = part5_simulate(N,figureName);
%% simulate with N/2
figureName='MPC_0_5_N';
[x_0_5_N,u_0_5_N,calc_time_0_5_N] = part5_simulate(N/2,figureName);
%% simulate with 2N
figureName='MPC_2_N';
[x_2_N,u_2_N,calc_time_2_N] = part5_simulate(2*N,figureName);
%% simulate with a big N
figureName='MPC_big_N';
bigN=50;
[x_big_N,u_big_N,calc_time_big_N] = part5_simulate(bigN,figureName);
%%
table=[...
    N/2,N,2*N,bigN; ...
    mean(calc_time_0_5_N),mean(calc_time_N),mean(calc_time_2_N), mean(calc_time_big_N)...
    ];

rowLabels = {'horizon', 't(s)'};
matrix2latex(table, './report/tables/MPC_time_table.tex', ...
    'rowLabels', rowLabels, ...
    'alignment', 'c', 'format', '%-6.2e', 'size', 'tiny')

J_N = total_sim_cost_MPC(x_N,u_N);
J_0_5_N = total_sim_cost_MPC(x_0_5_N,u_0_5_N);
J_2_N = total_sim_cost_MPC(x_2_N,u_2_N);

table=[...
    N/2,N,2*N; ...
    J_0_5_N,J_N, J_2_N; ...
    0 (J_N-J_0_5_N)/J_N (J_2_N-J_N)/J_2_N...
    ];

rowLabels = {'horizon', 'J','relative difference previous value of J'};
matrix2latex(table, './report/tables/MPC_cost_table.tex', ...
    'rowLabels', rowLabels, ...
    'alignment', 'c', 'format', '%-6.2e', 'size', 'tiny')