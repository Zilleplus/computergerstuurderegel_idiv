clear all;clc;close all;
load('trajectory_gk.mat');
load('discrete_model.mat');
%%
% split up y into its x and y coordinates
traject_x=y(1,:);
traject_y=y(2,:);
traject_theta=y(3,:);

fig=figure(1);clf;
% subplot(2,1,1);
% plot(traject_x(1:10),traject_y(1:10),'black');
% title('The first 10 points of the trajectory');
% xlabel('x');ylabel('y');

% subplot(2,1,2);
plot(traject_x,traject_y,'black');hold all;
title('Trajectory');
xlabel('x');ylabel('y');

saveas(fig,'./report/img/setpoint/traject.png');
%%

% build big-H
H=zeros(172*3,172*3);
% H(1:3,1:3)=Dd;
% build the matrix
buffer=zeros(3,172*3);
for k=1:171 % skip H0 as its special case
    buffer(:,(k-1)*3+1:(k-1)*3+3) = Cd*Ad^(172-k-1)*Bd;
%     buffer(:,(k-1)*3+1:(k-1)*3+3) = eye(3,3)*k;% Cd*Ad^(172-k)*Bd;
end
buffer(:,end-2:end) = Dd;

for i=1:172
    H((i-1)*3+1:(i-1)*3+3,1:(i-1)*3+3)=buffer(:,end-((i-1)*3+2):end);
end

% figure;spy(H);
% Build big-O
% O=zeros(172*3,6);
% for i=1:171
%     O((i-1)*3+1:(i-1)*3+3,:)=Cd*Ad^(i-1);
% end

% test matrices
H0=Dd;
H1=Cd*Ad^0*Bd;
H2=Cd*Ad^1*Bd;
H3=Cd*Ad^2*Bd;
H4=Cd*Ad^3*Bd;

H170=Cd*Ad^169*Bd;
H171=Cd*Ad^170*Bd;

% create w
w= reshape(y,[3*172,1]);
%%
% Solve the system
uopt=(H'*H)\H'*w;
uopt_reshape=reshape(uopt,[3,172]);

% now use the inputs and simulate the system
[output_simulation] = lsim(ss(Ad,Bd,Cd,Dd,Ts),uopt_reshape,(0:171)*Ts);
fig=figure(2);clf;

subplot(2,1,1);
plot(output_simulation(:,1),output_simulation(:,2),'O');hold all;
plot(traject_x,traject_y,'black');
xlabel('x');ylabel('y');
legend('simulated path','theoretical path');
title('Trajectory x_{ref}');

subplot(2,1,2);
plot(uopt_reshape(1,:));hold all;
plot(uopt_reshape(2,:));hold all;
plot(uopt_reshape(3,:));hold all;
title('input u_{ref}')
xlabel('t(s)');ylabel('u_{ref}');
legend('F_x','F_y','M');
saveas(fig,'./report/img/setpoint/simpleLeastSquares.png');

norm(output_simulation(:,1)-traject_x)
%%
R=eye(size(H))*10^-5; %10^-4 is the bil value
% R = triu(tril(repmat(diag([10^-4 10^-4 10^-4]),172)));
Q=eye(size(H))*10^0;
disp(['the condition of the matrix is: ' num2str(cond(R+H'*Q*H),'%10.5e')]);
disp(['the condition of the matrix without : ' num2str(cond(H'*H),'%10.5e')]);

uopt=(R+H'*Q*H)\(H'*Q*w);
uopt_reshape=reshape(uopt,[3,172]);
% 
[output_simulation,~,simulated_states] = lsim(ss(Ad,Bd,Cd,Dd,Ts),uopt_reshape,(0:171)*Ts,[0;0;0;0;0;0]);
fig=figure(3);clf;
subplot(2,1,1);
plot(simulated_states(:,1),simulated_states(:,2),'O'); hold all;
plot(traject_x,traject_y,'black');
title('Trajectory x_{ref}');
xlabel('x');ylabel('y');
legend('simulated path','theoretical path');

subplot(2,1,2);
plot(uopt_reshape(1,:));hold all;
plot(uopt_reshape(2,:));hold all;
plot(uopt_reshape(3,:));hold all;
title('input u_{ref}')
xlabel('t(s)');ylabel('u_{ref}');
legend('F_x','F_y','M');
saveas(fig,'./report/img/setpoint/conditionedLeastSquares.png');

norm(output_simulation(:,1)-traject_x);

x_ref=simulated_states';
u_ref=uopt_reshape;
save('reference_input_state','x_ref','u_ref');

disp(['Fxmax:' num2str(max(u_ref(1,:)))]);
disp(['Fymax:' num2str(max(u_ref(2,:)))]);
disp(['Mmax:' num2str(max(u_ref(3,:)),'%10.5e')]);