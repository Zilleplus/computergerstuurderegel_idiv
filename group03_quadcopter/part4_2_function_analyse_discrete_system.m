function [  ] = analyse_discrete_system( Ad,Bd,Cd,Dd,Ts )
    sys_discrete = ss(Ad,Bd,Cd,Dd,Ts);
    
    % where are the poles?
    disp(['the poles of the descrete system are:' ]);
    disp(eig(Ad));

    figure;
    pzmap(sys_discrete);

    % controllability / stabilisable
    C_= ctrb(Ad,Bd);
    disp(['The rank of the controllability matrix should be 12 in reality it is ' ...
        num2str(rank(C_))]);
    [V,~]=eig(Ad');
    disp(Bd'*V);

    % observability / detectable
    OO=obsv(Ad,Cd);
    disp(['The rank of the observability matrix should be 12 in reality it is ' ... 
        num2str(rank(OO))]);
    [V,~]=eig(Ad); 
    disp(Cd*V);

    % are there any transmission zeros?
    transmission_zeros_discrete = tzero(sys_discrete);
    disp(['there are ' num2str(size(transmission_zeros_discrete,1)) ...
        ' transmission zeros:' ]);
    disp(transmission_zeros_discrete);
end

