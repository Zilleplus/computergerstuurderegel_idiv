clear all;clc;close all;
load('trajectory_gk.mat');
load('reference_input_state.mat');
N=12;
%% With terminal cost
% simulate with N
figureName='MPC_N';
[x_N,u_N,calc_time_N] = part7_simulate(N,figureName);
% simulate with N
figureName='MPC_0_5_N';
[x_0_5_N,u_0_5_N,calc_time_0_5_N] = part7_simulate(0.5*N,figureName);
% simulate with N
figureName='MPC_2N';
[x_2_N,u_2_N,calc_time_2_N] = part7_simulate(2*N,figureName);
% get the results from 1.5
figureName='NOSAVE';
[x_N_old,u_N_old,calc_time_N_old] = part5_simulate(N,figureName);
[x_0_5_N_old,u_0_5_N_old,calc_time_0_5_N_old] = part5_simulate(0.5*N,figureName);
[x_2_N_old,u_2_N_old,calc_time_2_N_old] = part5_simulate(2*N,figureName);
%% compare the different horizons
fig=figure;
plot(x_N(1,:),x_N(2,:),'.');hold all;
plot(x_0_5_N(1,:),x_0_5_N(2,:),'.');hold all;
plot(x_2_N(1,:),x_2_N(2,:),'.');hold all;

plot(x_ref(1,:),x_ref(2,:),'+');
plot(y(1,:),y(2,:),'black');

legend('horizon=N', 'horizon=0.5N','horizon=2N','reference path', ...
    'theoretical path', 'Location', 'southeast');
xlabel('x');ylabel('y');
xlim([1 10]);
ylim([3 11]);

saveas(fig,'./report/img/MPC_term_cost/compare_ROI.png');
%% draw figure for each horizon
region_of_interest_x = [1 10];
region_of_interest_y = [3 11];

fig=figure;
plot(x_N(1,:),x_N(2,:),'.');hold all;
plot(x_N_old(1,:),x_N_old(2,:),'.');hold all;

plot(x_ref(1,:),x_ref(2,:),'+');
plot(y(1,:),y(2,:),'black');
legend('with terminal cost', 'without terminal cost','reference','theoretical', ...
    'Location', 'southeast');
title(['horizon=' num2str(N)]);
xlabel('x');ylabel('y');
xlim(region_of_interest_x);
ylim(region_of_interest_y);

saveas(fig,'./report/img/MPC_term_cost/compare_ROI_N.png');

fig=figure;
plot(x_0_5_N(1,:),x_0_5_N(2,:),'.');hold all;
plot(x_0_5_N_old(1,:),x_0_5_N_old(2,:),'.');hold all;

plot(x_ref(1,:),x_ref(2,:),'+');
plot(y(1,:),y(2,:),'black');
legend('with terminal cost', 'without terminal cost','reference','theoretical', ...
    'Location', 'southeast');
title(['horizon=' num2str(N/2)]);
xlabel('x');ylabel('y');
xlim(region_of_interest_x);
ylim(region_of_interest_y);

saveas(fig,'./report/img/MPC_term_cost/compare_ROI_0_5_N.png');

fig=figure;
plot(x_2_N(1,:),x_2_N(2,:),'.');hold all;
plot(x_2_N_old(1,:),x_2_N_old(2,:),'.');hold all;

plot(x_ref(1,:),x_ref(2,:),'+');
plot(y(1,:),y(2,:),'black');
legend('with terminal cost', 'without terminal cost','reference','theoretical', ...
    'Location', 'southeast');
title(['horizon=' num2str(N*2)]);
xlabel('x');ylabel('y');
xlim(region_of_interest_x);
ylim(region_of_interest_y);

saveas(fig,'./report/img/MPC_term_cost/compare_ROI_2_N.png');
%% total Simulation cost

 