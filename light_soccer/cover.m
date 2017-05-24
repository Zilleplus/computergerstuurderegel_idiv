function [ xl, xr,zl,zr,range_penality] = cover( goalkeeper,attacker )
    % goalkeeper should contain: [x_k y_k theta]
    x_k=goalkeeper(1);
    y_k=goalkeeper(2);
    theta=goalkeeper(3);
    % attacker should contain [x_a y_a]
    x_a=attacker(1);
    y_a=attacker(2);
    % range penalty, a penalty because the parameters are out of range
    range_penality=0;
    % fixed parameter 
    d=1;
    goalWidth=7.32;
    penaltyWidth=16.5+7.32+7.32;
    penaltylength=16.5;
    % ----------------------------------------------------
    % safety procedure , goalkeep cannot leave the penalty
    % ----------------------------------------------------
    if(x_k>penaltyWidth/2)
        range_penality=range_penality+abs(x_k-penaltyWidth/2); % punish
        x_k=penaltyWidth/2; % fix it
    end
    if(x_k<-penaltyWidth/2)
        range_penality=range_penality+abs(x_k-penaltyWidth/2); % punish
        x_k=-penaltyWidth/2; % fix it
    end
    if(y_k>penaltylength)
        range_penality=range_penality+abs(y_k-penaltylength); % punish
        y_k=penaltylength; % fix it
    end
    if(y_k<0)
        range_penality=range_penality+abs(y_k); % punish
        y_k=0; % fix it
    end
    % ----------------------------------------------------
    delta_x=cos(theta)*d;
    delta_y=sin(theta)*d;

    % find both sides of the keeper, p1 and p2
    x_p1=x_k+delta_x;
    y_p1=y_k+delta_y;

    x_p2=x_k-delta_x;
    y_p2=y_k-delta_y;

    % find where a straigh line from the attackers position towards p1/p2 
    % cuts the x-axis

    a1=(y_a-y_p1)/(x_a-x_p1);
    a2=(y_a-y_p2)/(x_a-x_p2);

    z1=x_p1-(1/a1)*y_p1;
    z2=x_p2-(1/a2)*y_p2;

    % put the right coordinate with the right side
    if(z1<z2)
        zl=z1;zr=z2;
    else
        zl=z2;zr=z1;
    end
    
    xl=abs(zl+goalWidth/2);
    xr=abs(goalWidth/2-zr);
end