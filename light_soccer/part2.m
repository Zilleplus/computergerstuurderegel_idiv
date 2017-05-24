clc;clear all;close all;

Ts = 0.15;
N = 12; % prediction  horizon

rho = 20;
m = 90;
I = 1.8;

Fmax = 180;
Mmax = 10;

save('upper_limits','Fmax','Mmax');

TFINAL=4;

% Continous system
A = zeros(6,6);
B = zeros(6,3);
C = zeros(3,6);
D = zeros(3,3);

A(1:3,4:6)=eye(3);
A(4,4) = -rho/m;
A(5,5) = -rho/m;
A(6,6) = -0.01/I;

B(4,1) = 1/m;
B(5,2) = 1/m;
B(6,3) = 1/I;

C = [eye(3) zeros(3,3)];  

ss_cont = ss(A,B,C,D);

%% Discretization Euler
Ad = eye(6) + Ts*A;
Bd = Ts*B;
Cd = C;
Dd = D;

part2_function_analyse_discrete_system(Ad,Bd,Cd,Dd,Ts);
ss_Euler = ss(Ad,Bd,Cd,Dd,Ts);

fig=figure(1);clf;
step(ss_Euler,TFINAL);
title('step response Euler');
saveas(fig,'./report/img/keeperModel/euler_step.eps');
%% Discretization back rect
Ad = inv(eye(6) - Ts*A);
Bd = Ad * Ts*B;
Cd = C * Ad;
Dd = D + Cd * B*Ts;

part2_function_analyse_discrete_system(Ad,Bd,Cd,Dd,Ts);
ss_backr = ss(Ad,Bd,Cd,Dd,Ts);

fig=figure(2);clf
step(ss_backr,TFINAL);
title('step response zero on hold');
saveas(fig,'./report/img/keeperModel/backr_step.eps');
save('discrete_model_backRect','Ad','Bd','Cd','Dd','Ts');
%% Discretization ZOH
ss_ZOH = c2d(ss_cont,Ts,'zoh');

part2_function_analyse_discrete_system(Ad,Bd,Cd,Dd,Ts);
[Ad,Bd,Cd,Dd] = ssdata(ss_ZOH);

fig=figure(3);clf;
step(ss_ZOH,TFINAL);
title('step response zero on hold');
saveas(fig,'./report/img/keeperModel/zoh_step.eps');
%% Discretization bilinear
% define descrete matrices bilear method:
Ad= inv(eye(size(A))-A.*(Ts/2))*(eye(size(A))+A.*(Ts/2));
Bd= inv(eye(size(A))-A.*(Ts/2))*B*Ts;
Cd= C*inv(eye(size(A))-A.*(Ts/2));
Dd= D + C*inv(eye(size(A))-A.*(Ts/2))*B*(Ts/2);

part2_function_analyse_discrete_system(Ad,Bd,Cd,Dd,Ts);
ss_bil = ss(Ad,Bd,Cd,Dd,Ts);

fig=figure(4);clf;
step(ss_bil,TFINAL);
title('step response bilinear transformation');
saveas(fig,'./report/img/keeperModel/bil_step.eps');

% pick this a discretization
save('discrete_model','Ad','Bd','Cd','Dd','Ts');
%%
fig=figure(5);clf;
step(ss_cont,TFINAL);
title('step response continious system')
saveas(fig,'./report/img/keeperModel/cont_step.eps');