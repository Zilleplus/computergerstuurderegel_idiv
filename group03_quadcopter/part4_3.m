% load the lin model:
run('part4_1_lin_approx.m');
clc;

% define descrete matrices for euler:
Adiscrete= inv(eye(size(A))-A.*(Ts/2))*(eye(size(A))+A.*(Ts/2));
Bdiscrete= inv(eye(size(A))-A.*(Ts/2))*B*Ts;
Cdiscrete= C*inv(eye(size(A))-A.*(Ts/2));
Ddiscrete= D + C*inv(eye(size(A))-A.*(Ts/2))*B*(Ts/2);

disp('ready to calculate the controllers !');

%% implement a full state controller:
% define an Q and an R
ss_discrete=ss(Adiscrete,Bdiscrete,Cdiscrete,Ddiscrete,Ts);
Q=eye(size(A,2));
Q(3,3)=10^2;%10^2; %10^2 is the magic number :-)
R=eye(size(B,2));
R=R*0.1;

[K,S,E] = lqr(ss_discrete,Q,R,[]);

right=[zeros(12,6);eye(6,6)];
left=[Adiscrete-eye(size(A)) Bdiscrete; Cdiscrete Ddiscrete];
sol=left\right;

Nx=sol(1:12,:);
Nu=sol(13:end,:);
%% implement an LQR controller with integral action
% define the augemented system with the integrator variables
II=zeros(6,12);
II(1:3,1:3) = eye(3);

Anew=[ 
        eye(3) Cdiscrete(1:3,:);
        zeros(12,3) Adiscrete
      ];
Bnew=[Ddiscrete(1:3,:);Bdiscrete];
Cnew=[
    zeros(6,3) Cdiscrete
    ];

Dnew=Ddiscrete;

ss_augmented=ss(Anew,Bnew,Cnew,Dnew,Ts);

Q=eye(size(Anew,2));
% integrators
Q(1,1)=10^3;
Q(2,2)=10^3;
Q(3,3)=10^3; % adjusted

% normal 
Q(4,4)=10^2;
Q(5,5)=10^2;
Q(6,6)=10^2;
 
% % speed
Q(7,7)=10^4;
Q(8,8)=10^4;

R=eye(size(Bnew,2));
R=R.*50;

[K,~,~] = lqr(ss_augmented,Q,R,[]);

K1=K(:,1:3);
K0=K(:,4:15);

% variable using in simulink:
I_filter = eye(3,6);

%% adjusted full-state controller for 0.1 load
% define an Q and an R
ss_discrete=ss(Adiscrete,Bdiscrete,Cdiscrete,Ddiscrete,Ts);
Q=eye(size(A,2));
Q(3,3)=10^2;%10^2; %10^2 is the magic number :-)
R=eye(size(B,2));
R=R*0.1;

[K,S,E] = lqr(ss_discrete,Q,R,[]);

right=[zeros(12,6);eye(6,6)];
left=[Adiscrete-eye(size(A)) Bdiscrete; Cdiscrete Ddiscrete];
sol=left\right;

Nx=sol(1:12,:);
Nu=sol(13:end,:);
%% Design the kallman filter
%  lqe  Kalman estimator design for continuous-time systems.
%     Given the system
%         x = Ax + Bu + Gw            {State equation}
%         y = Cx + Du + v             {Measurements}
%     with: E{ww'} = Q,    E{vv'} = R,    E{wv'} = N
% [L,P,E] = lqe(A,G,C,Q,R,N)

% output noise:
R=diag([2.5*10^-5 2.5*10^-5 2.5*10^-5 7.57*10^-5 7.57*10^-5 7.57*10^-5]);
% state noise:
Q=eye(12);
% w=12x1 and v=6x1 -> E{wv'} = N
N=zeros(12,6);
G=eye(12,12);

[L,~,~] = lqe(Adiscrete,G,Cdiscrete,Q,R,N);