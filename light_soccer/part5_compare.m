clc;clear all;close all;
%%
load('discrete_model.mat'); % load the discrete model
% load('discrete_model_backRect.mat');
sys=ss(Ad,Bd,Cd,Dd,Ts);
load('reference_input_state.mat'); % load u_ref and x_ref
load('upper_limits.mat');
load('trajectory_gk.mat');
%%
% prepare reference data for simulink
timeStamp=[0:171]*Ts;
x_ref_simu = [timeStamp' x_ref'];
u_ref_simu = [timeStamp' u_ref'];

R = eye(3,3)*10^-4;
Q = diag([ones(1,3) zeros(1,3)]);

[K,~,~] = lqr(sys,Q,R);

sim('LQR.slx');

%% MPC

N=12;

figureName='NOSAVE';
[x_N,u_N,calc_time_N] = part5_simulate(N,figureName);

figureName='NOSAVE';
[x_0_5_N,u_0_5_N,calc_time_0_5_N] = part5_simulate(N/2,figureName);

figureName='NOSAVE';
[x_2_N,u_2_N,calc_time_2_N] = part5_simulate(2*N,figureName);

close all;
%%
fig=figure;
plot(sim_data.Data(:,1),sim_data.Data(:,2),'.');hold all;
plot(x_N(1,:),x_N(2,:),'.');hold all;
plot(x_2_N(1,:),x_2_N(2,:),'.');hold all;
plot(x_0_5_N(1,:),x_0_5_N(2,:),'.');hold all;
plot(x_ref(1,:),x_ref(2,:),'+');
plot(y(1,:),y(2,:),'black');
legend('LQR2','MPC N','MPC 2N','MPC 0.5*N','reference','theoretical');
xlabel('x');ylabel('y');
saveas(fig,'./report/img/MPC/compare.png');
%%
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
saveas(fig,'./report/img/MPC/compare_horizon.png');

