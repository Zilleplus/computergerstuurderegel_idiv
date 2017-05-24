clc;clear all;close all;

Ts = 0.15;
N = 12; % prediction  horizon

rho = 20;
m = 90;
I = 1.8;

Fmax = 180;
Mmax = 10;

save('upper_limits','Fmax','Mmax');

TFINAL=2;

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

% Continue system
[Y_cont,T_cont] = step(ss_cont,TFINAL);

% Discretization Euler
Ad = eye(6) + Ts*A;
Bd = Ts*B;
Cd = C;
Dd = D;

ss_Euler = ss(Ad,Bd,Cd,Dd,Ts);
[Y_euler,T_euler] = step(ss_Euler,TFINAL);

% Discretization back rect
Ad = inv(eye(6) - Ts*A);
Bd = Ad * Ts*B;
Cd = C * Ad;
Dd = D + Cd * B*Ts;

ss_backr = ss(Ad,Bd,Cd,Dd,Ts);
[Y_backR,T_backR] = step(ss_backr,TFINAL);

% Discretization ZOH
ss_ZOH = c2d(ss_cont,Ts,'zoh');
[Y_zoh,T_zoh] = step(ss_ZOH,TFINAL);

% Define descrete matrices bilear method:
Ad= inv(eye(size(A))-A.*(Ts/2))*(eye(size(A))+A.*(Ts/2));
Bd= inv(eye(size(A))-A.*(Ts/2))*B*Ts;
Cd= C*inv(eye(size(A))-A.*(Ts/2));
Dd= D + C*inv(eye(size(A))-A.*(Ts/2))*B*(Ts/2);

ss_bil = ss(Ad,Bd,Cd,Dd,Ts);
[Y_bil,T_bil] = step(ss_bil,TFINAL);

% only create the diagonal figures as described in the report
ylabels={'x', 'y', '\theta'};
for i=1:3
    fig=figure(i);clf;
    plot(T_cont,Y_cont(:,i,i),'O');hold all;
    plot(T_zoh,Y_zoh(:,i,i));hold all;
    plot(T_euler,Y_euler(:,i,i));hold all;
    plot(T_backR,Y_backR(:,i,i));hold all;
    plot(T_bil,Y_bil(:,i,i));hold all;
    
    ylabel(ylabels(i));
    xlabel('t(s)');

    legend('continue','zero-order hold','Euler',...
        'backward rectangular','bilinear');
    
    saveas(fig,['./report/img/keeperModel/step_impulse_total_' ...
        num2str(i) '.png']);
end
