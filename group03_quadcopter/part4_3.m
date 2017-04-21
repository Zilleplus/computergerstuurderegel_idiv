ss_discrete=ss(Adiscrete,Bdiscrete,Cdiscrete,Ddiscrete,Ts);
%% implement a full state controller:
% define an Q and an R
Q=eye(size(A,2));
Q(3,3)=10^7; %10^7 is the magic number :-)
R=eye(size(B,2));
[K,S,E] = lqr(ss_discrete,Q,R,[]);

right=[zeros(12,6);eye(6,6)];
left=[Adiscrete-eye(size(A)) Bdiscrete; Cdiscrete Ddiscrete];
sol=left\right;

Nx=sol(1:12,:);
Nu=sol(13:end,:);
%% implement an LQR controller with integral action
