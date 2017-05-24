clc;clear all;close all;
%%
load('discrete_model.mat'); % load the discrete model
% load('discrete_model_backRect.mat');
sys=ss(Ad,Bd,Cd,Dd,Ts);
load('reference_input_state.mat'); % load u_ref and x_ref
load('upper_limits.mat');
load('trajectory_gk.mat');

% prepare reference data for simulink
timeStamp=[0:171]*Ts;
x_ref_simu = [timeStamp' x_ref'];
u_ref_simu = [timeStamp' u_ref'];
%%
R = eye(3,3)*10^-4;
Q = eye(6,6);

[K,~,~] = lqr(sys,Q,R);
K1=K; % save K to compare with K2

sim('LQR.slx');

part4_plot(x_ref,y,input_data,sim_data,'LQR1');

LQR1_sim_data=sim_data;
LQR1_sim_input=input_data;

%%
R = eye(3,3)*10^-4;
Q = diag([ones(1,3) zeros(1,3)]);

[K,~,~] = lqr(sys,Q,R);
K2=K; % save K to compare with K1

sim('LQR.slx');

part4_plot(x_ref,y,input_data,sim_data,'LQR2');

LQR2_sim_data=sim_data;
LQR2_sim_input=input_data;

%% compare the two
fig=figure;
subplot(2,1,1);
plot(LQR1_sim_data.Data(:,1),LQR1_sim_data.Data(:,2));hold all;
plot(LQR2_sim_data.Data(:,1),LQR2_sim_data.Data(:,2));
legend('LQR1','LQR2');
title('total trajectory');
subplot(2,1,2);
plot(LQR1_sim_data.Data(:,1),LQR1_sim_data.Data(:,2));hold all;
plot(LQR2_sim_data.Data(:,1),LQR2_sim_data.Data(:,2));
xlim([-10.35 -9.85 ]);
ylim([12 20 ]);
legend('LQR1','LQR2');
title('zoomed in on corner:')
saveas(fig,'./report/img/LQR/LQR_compare.png');
%% Alternative approach
% R = eye(3,3)*10^2;
% Q = diag([ones(1,3) zeros(1,3)]);
% 
% [K,~,~] = lqr(sys,Q,R);
% K3=K; % save K to compare with K1
% 
% sim('LQR.slx');
% 
% part4_plot(x_ref,y,input_data,sim_data,'LQRincreasedR');
%%
J_LQR1=total_sim_cost(LQR1_sim_data.Data',LQR1_sim_input.Data',R,Q);
J_LQR2=total_sim_cost(LQR1_sim_data.Data',LQR2_sim_input.Data',R,Q);

table=[...
    J_LQR1, J_LQR2
    ];
columnLabels = {'$LQR_1$', '$LQR_2$'};
matrix2latex(table, './report/tables/LQR_cost.tex', ...
    'columnLabels', columnLabels, ...
    'alignment', 'c', 'format', '%-8.4e', 'size', 'tiny');
