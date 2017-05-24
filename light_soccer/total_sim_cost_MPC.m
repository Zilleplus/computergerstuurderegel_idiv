function J = total_sim_cost_MPC( x,u )
    R = eye(3,3)*10^-4;
    Q = diag([1 1 1 0 0 0]);
    J=0;
    for i=1:size(x,2)
        J = J + (x(:,i)'*Q*x(:,i));
    end
    for i=1:size(u,2)
        J = J + (u(:,i)'*R*u(:,i));
    end
end

