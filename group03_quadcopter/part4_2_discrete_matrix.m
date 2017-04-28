% matrices A,B,C and D should be available in the workspace
% the continue system:
sys_continue = ss(A,B,C,D);
% parameters:


%% Euler
clc;
% define descrete matrices for euler:
Ad=(eye(size(A))+A.*Ts);
Bd=B*Ts;
Cd=C;
Dd=D;

part4_2_function_analyse_discrete_system(Ad,Bd,Cd,Dd,Ts);

%% Bilinear
clc;
% define descrete matrices for euler:
Ad= inv(eye(size(A))-A.*(Ts/2))*(eye(size(A))+A.*(Ts/2));
Bd= inv(eye(size(A))-A.*(Ts/2))*B*Ts;
Cd= C*inv(eye(size(A))-A.*(Ts/2));
Dd= D + C*inv(eye(size(A))-A.*(Ts/2))*B*(Ts/2);

% this is the prefered choice more info in the report
Adiscrete=Ad;
Bdiscrete=Bd;
Cdiscrete=Cd;
Ddiscrete=Dd;
%%
part4_2_function_analyse_discrete_system(Ad,Bd,Cd,Dd,Ts);
%% zero and hold
clc;
% define descrete matrices for euler:
Ad=exp(A*Ts);
Bd=inv(A)*(exp(A*Ts)-eye(size(A)))*B;
Cd=C;
Dd=D;
disp(['!! as A is not invertible the zero and hold method cannot be used !!'])
% analyse_discrete_system(Ad,Bd,Cd,Dd,Ts);

%%