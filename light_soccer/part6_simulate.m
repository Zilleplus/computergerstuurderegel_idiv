%  return:
% x_sim : simulation states
% u_feedback : the inputs applied each iteration
% calc_time : calculation for each iteration
% N=12;
% gainF=100;
% xmax=16.5+(7.32/2);
% ymax=16.5;
function  [x_sim,u_feedback,calc_time] = part6_simulate(N,figureName,xmax,ymax,gainF)
    % load required variables
    load('discrete_model.mat');
    load('upper_limits.mat');
    load('trajectory_gk.mat');
    size_reference=size(y,2);
    load('reference_input_state.mat');
    
    Fmax=Fmax*gainF;

    % setup static matrices 
    R = eye(3,3)*10^-4;
    Q = diag([1 1 1 0 0 0]);

    H = 2*diag([repmat(diag(Q),N,1);repmat(diag(R),N,1)]);

    % inequalities
    % inequalities -> keep the inputs within a certain value
    Ai_input = [ zeros(N*6),[ eye(N*3); -eye(N*3) ] ];
    bi_input = [ Fmax , Fmax , Mmax ]';
    bi_input = repmat(bi_input,2*N,1);

    % inequalities -> keep the states within a certain value
    % create a matrix to contain all the states
    Ai_state_full = [
            eye(N*6) zeros(N*6,N*3);
            -eye(N*6) zeros(N*6,N*3);
                ];
    %
    % find the indexes for the states that are used in constraint
    index_x = ([1:N] -1).*6 +1;
    index_y = ([1:N] -1).*6 +2;
    
    index_row_Ai=zeros(1,length(index_y)*2);
    index_row_Ai(1,1:2:length(index_y)*2)=index_x;
    index_row_Ai(1,2:2:length(index_y)*2)=index_y;

    % build the Ai matrix with the proper rows
    Ai_state=Ai_state_full([index_row_Ai index_row_Ai+N*6],:);
    %
    bi_state = repmat([xmax ; ymax],N,1);  
    bi_state = [bi_state ;repmat([xmax ; 0],N,1)];
    
    Ai=[Ai_input;Ai_state];
    bi=[bi_input;bi_state];
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
        [x_opt,~,exitflag] = quadprog(H,f,Ai,bi,Ae,be);
        calc_time(i)=toc;
        if(exitflag~=1)
            disp(['solution is not feaseable break the simulation on iteration:' ...
                num2str(i)]);
            break;
        end
        % the first halve of x_u exist out of states so skip 6*N elements
        % u= u(k+1)so increase it even further by 4 to get second set of u
        u = x_opt(6*N+1:6*N+3);
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
    plot([-15 10],[ymax ymax],'-');
    title(['trajectory horizon=' num2str(N)]);
    legend('traject simulation controller','reference trajectory','theoretical trajectory','y_{max}');
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
        saveas(fig,['./report/img/MPC_state_constraint/' figureName '_traj_input.png']);
    end
end

