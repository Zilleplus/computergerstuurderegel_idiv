clear all;clc;close all;
%%
% loading constants
m=0.5;
L=0.25;
k=3*10^-6;
b=10^-7;
g=9.81;
kd=0.25;
Ixx=5*10^-3;
Iyy=5*10^-3;
Izz=10^-3;
cm=10^4;
load references_03;
u_eq = [1; 1; 1; 1]*(m*g)/(k*cm*4);
% u_eq = [1; 1; 1; 1]*12;
% set the unitial state:
x0_quadcopter=zeros(12,1);

Ts=0.05;

%%
% defining the lin model:

A = zeros(12,12);

A(1,4)=1;
A(2,5)=1;
A(3,6)=1;

A(4,4)=-kd/m; A(4,8)=g;
A(5,5)=-kd/m; A(5,7)=-g;
A(6,6)=-kd/m;

A(7:9,10:12) = eye(3,3);

disp(A);

B = zeros(12,4);

B(6,:)=[1 1 1 1].*(k*cm/m);

B(10,1)=(L*k*cm)/(Ixx);
B(10,3)=-(L*k*cm)/(Ixx);
B(11,2)=(L*k*cm)/(Iyy);
B(11,4)=-(L*k*cm)/(Iyy);

B(12,1)=(b*cm)/(Izz);
B(12,2)=-(b*cm)/(Izz);
B(12,3)=(b*cm)/(Izz);
B(12,4)=-(b*cm)/(Izz);

disp(B);

C=zeros(6,12);
C(1:3,1:3)=eye(3,3);
C(4:6,7:9)=eye(3,3);

D=zeros(6,4);


%% uncomment to generate the step impulse
% sim('part4_1_lin_approximation');
% fig=figure;
% subplot(1,2,1);
% for i=1:6
%     plot(lin_model_stepresponse.Time,lin_model_stepresponse.Data(:,i));hold all;
% end
% title('linear model');
% xlabel('t(s)');
% legend('x','y','z','\phi','\theta','\gamma','location','northwest');
% subplot(1,2,2);
% for i=1:6
%     plot(non_lin_model_stepresponse.Time,non_lin_model_stepresponse.Data(:,i));hold all;
% end
% title('non-linear model');
% xlabel('t(s)');
% legend('x','y','z','\phi','\theta','\gamma','location','northwest');
% 
% saveas(fig,'./report/img/lin_approx/step_all.png');
