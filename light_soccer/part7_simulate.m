%  return:
% x_sim : simulation states
% u_feedback : the inputs applied each iteration
% calc_time : calculation for each iteration
function  [x_sim,u_feedback,calc_time] = part7_simulate( N,figureName )
    % load required variables
    load('discrete_model.mat');
    load('upper_limits.mat');
    load('trajectory_gk.mat');
    size_reference=size(y,2);
    load('reference_input_state.mat');

    % setup static matrices 
    R = eye(3,3)*10^-4;
    Q = diag([1 1 1 0 0 0]);
    [~,S]=dlqr(Ad,Bd,Q,R);

    H = 2*diag([repmat(diag(Q),N,1);repmat(diag(R),N,1)]);
    % terminal cost: replace matrix in H
    H(6*(N-1)+1:6*(N-1)+6,6*(N-1)+1:6*(N-1)+6) = S;

    % inequalities
    Ai = [ zeros(N*6),[ eye(N*3); -eye(N*3) ] ];
    bi = [ Fmax , Fmax , Mmax ]';
    bi = repmat(bi,2*N,1);
    % -----------
    % equalities
    Ae=[];Be=[];
    for i=1:N
        Be = blkdiag(Be,-Bd);
        Ae = blkdiag(Ae,-Ad);
    end
    Ae =Ae*(diag(ones((N-1)*6,1),-6))+eye(6*N);
    Ae = [Ae,Be];
    % -----------

    x=zeros(6,1); % start the simulation from [0 0 0 0 0 0]
    % y_mpc = zeros((Ny-N),3);
    u_feedback = zeros(3,(size_reference));
    x_sim = zeros(6,(size_reference));
    calc_time=zeros(1,size_reference-N);
    for i=1:(size_reference-N+1) 
        % define the vector f
        f = -H*[ 
            reshape(x_ref(:,i:N+i-1),N*6,1); % 6 states
            reshape(u_ref(:,i:N+i-1),N*3,1) % 3 inputs
            ];
        be=zeros(1,N*6); % 6 states so..
        be(1:6) = Ad*x;
        tic;
        % solve the optimization problem with quadprog
        x_opt = quadprog(H,f,Ai,bi,Ae,be);
        calc_time(i)=toc;
        % the first halve of x_u exist out of states so skip 6*N elements
        % u= u(k+1)so increase it even further by 4 to get second set of u
        u = x_opt(6*N+4:6*N+6);
        u_feedback(:,i) = u; % save the input 
        x_sim(:,i) = x; % save the state

        x = Ad*x+Bd*u;

        % if this is the last loop, simulate a bit further with given u's
        if(i==size_reference-N+1)
            % get the predicted u's
            u_remaining_simulations=x_opt(6*N+7:end);
            remaining_simulation_length = size( u_remaining_simulations,1)/3;

            % for convenience reshape the matrix
            u_remaining_simulations = ...
                reshape(u_remaining_simulations,3,remaining_simulation_length);

            for i_remaining=1:remaining_simulation_length
                u = u_remaining_simulations(:,i_remaining);

                u_feedback(:,i+i_remaining) = u; % save u
                x_sim(:,i+i_remaining) = x; % save x
                x = Ad*x+Bd*u; % simulate
            end
            x_sim(:,i+remaining_simulation_length+1) = x;
        end
    end

    % visualize the simulation results
    fig=figure;clf;
    subplot(2,1,1);
    plot(x_sim(1,:),x_sim(2,:),'O'); hold all;
    plot(x_ref(1,:),x_ref(2,:),'.'); hold all;
    plot(y(1,:),y(2,:),'black'); hold all;
    title(['trajectory horizon=' num2str(N)]);
    legend('traject simulation controller','reference trajectory','theoretical trajectory');
    xlabel('x');ylabel('y');

    subplot(2,1,2);
    time=[0:size_reference-1]*Ts;
    plot(time,u_feedback(1,:)); hold all;
    plot(time,u_feedback(2,:)); hold all;
    plot(time,u_feedback(3,:)); hold all;
    title('input goal keeper in function of time');
    legend('F_x','F_y','m');
    xlabel('t(s)');ylabel('u');
    
    if(~strcmp(figureName,'NOSAVE'))
        saveas(fig,['./report/img/MPC_term_cost/' figureName '_traj_input.png']);
    end
    
end

