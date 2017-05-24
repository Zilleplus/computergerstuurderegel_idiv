function J = total_sim_cost( x,u,R,Q )
    J=0;
    for i=1:size(x,2)
        J = J + (x(:,i)'*Q*x(:,i));
    end
    for i=1:size(u,2)
        J = J + (u(:,i)'*R*u(:,i));
    end
end