clear all;
close all;

%%%% FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), x(k+1)=x(k)+u_y(k) %%%%

% Data change
n_lim = 5;

% Constraints on state space
bound_x = [1 n_lim];
bound_y = [1 n_lim];
bound = [bound_x; bound_y];

%Constraints on the input space
bound_u = [-2 0; -1 0; 1 0; 2 0; 0 -2; 0 -1; 0 1; 0 2];
n_u = numel(bound_u)/2;

% Space discretization
n_x = n_lim;
n_y = n_lim;

% size of the interval
d_x=(bound_x(:,2)-bound_x(:,1))./(n_x);
d_y=(bound_y(:,2)-bound_y(:,1))./(n_y);
d_states = [d_x;d_y];

% Number of robots
n_r = 2;
n_states = [2; 2];
n_inputs = [1; 1]; 

% states_data contains discretization data for all the robots states
states_data = [[n_x n_y]; [n_x n_y]];

% inputs_data contains discretization data for all the robots inputs
inputs_data = [[n_u]; [n_u]];

%%%%% Barrier Certificate = norm(x_i-x_j)-d-L*eta_max %%%%%
d=1; % Distance to be maintained
L=1; % Parameter for abstraction of barrier
eta_max = max((bound_x(2)-bound_x(1)+1)/n_x, (bound_y(2)-bound_y(1)+1)/n_y);

load('Controlled_System1');
load('Controlled_System2');
load('Target_1');
load('Target_2');

%load('Delta3');
%load('Delta4');
%load('Delta5');

tic
%%% Composition of Controlled system
Delta = logical(sparse((n_x*n_y*n_u).^n_r, (n_x*n_y).^n_r));

p2 = waitbar(0,'Abstraction in progress...');

delta1_idxs = find(controlled_Delta1);
delta2_idxs = find(controlled_Delta2);

for d1_idx=1:size(delta1_idxs, 1)
    waitbar(d1_idx/size(delta1_idxs, 1), p2, sprintf('Computing the abstraction of the source subsystem - %2.2f', d1_idx/size(delta1_idxs, 1)))
    [r1_c, r1_s] = ind2sub(size(controlled_Delta1), delta1_idxs(d1_idx));
    for d2_idx=1:size(delta2_idxs, 1)
        [r2_c, r2_s] = ind2sub(size(controlled_Delta2), delta2_idxs(d2_idx));
        [i1, i2, h1] = get_current_pose(r1_c, n_x, n_y, n_u);
        [i3, i4, h2] = get_current_pose(r2_c, n_x, n_y, n_u);
        [ip1, ip2] = get_successor_pose(r1_s, n_x, n_y);
        [ip3, ip4] = get_successor_pose(r2_s, n_x, n_y);
       
        if(norm([i1 i2] - [i3 i4])-d-L*eta_max>=0)
            B_prime = norm([ip1 ip2] - [ip3 ip4]) - norm([i1 i2] - [i3 i4]);
            if (B_prime >= -1*(norm([i1 i2] - [i3 i4])-d-L*eta_max))
                Delta((i1-1)*n_y*n_x*n_y*n_u*n_u+(i2-1)*n_x*n_y*n_u*n_u+(i3-1)*n_y*n_u*n_u+(i4-1)*n_u*n_u+(h1-1)*n_u+h2,(ip1-1)*n_y*n_x*n_y+(ip2-1)*n_x*n_y+(ip3-1)*n_y+ip4)=1;
            end
        end
    end
end
toc

tic
% Controller Synthesis
Cont=logical(zeros((n_x)*n_y*n_x*n_y,(n_u)*n_u)); % create the matrix structure (defined only with 0,1)
succ_idxs = find(Delta);

winning_domain_new = zeros(size(succ_idxs, 1), 6);

for succ_idx=1:size(succ_idxs, 1)
%     waitbar(succ_idx/size(succ_idxs, 1), p2, sprintf('Computing the Controller Synthesis - %2.2f', succ_idx/size(succ_idxs, 1)))
    [r_c, r_s] = ind2sub(size(Delta), succ_idxs(succ_idx));
    points = get_composed_current_pose(r_c, n_r, n_states, n_inputs, states_data, inputs_data);
    i = (points(6)-1)*n_y*n_x*n_y+(points(5)-1)*n_x*n_y+(points(4)-1)*n_y+points(3);
    h = (points(2)-1)*n_u+points(1);
    Cont(i, h) = 1;
    winning_domain_new(succ_idx, :) = [points(6), points(5), points(4), points(3), points(2), points(1)];
end


ContA=Cont;

iter=0;
%%%%% FOR REACHABILITY %%%%%
Contp = logical(zeros((n_x)*n_y*n_x*n_y,(n_u)*n_u));
%%% Reachability Specification. Consider a region
pReach_states=[(t_r1(1)-1)*n_y*n_x*n_y+(t_r1(2)-1)*n_x*n_y+(t_r2(1)-1)*n_y+t_r2(2)];
reach_state_target=pReach_states;
while ~all(Cont(:)==Contp(:))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter)
    Contp=Cont;
    
    mode_idxs = find(ContA);
    for mode_idx=1:size(mode_idxs, 1)
        [mode_r, mode_c] = ind2sub(size(ContA), mode_idxs(mode_idx));
        succ=find(Delta((mode_r-1)*n_u*n_u+mode_c,:));
        if ~ismember(mode_r,pReach_states) % there is something here
            if ~(ismember(succ,pReach_states))
                Cont(mode_r,mode_c)=0;
            else
                Cont(mode_r,mode_c)=1;
                pReach_states = cat(2, pReach_states, mode_r);
            end
        end
    end
end
close(p2)
toc
disp(' ')

% Simulation
% curr_pose: [[p_x1, p_y1, p_x2, p_y2]]
curr_pose = [1, 1, 4, 4];
poses = curr_pose;

% Current pose index
curr_pos_idx = (curr_pose(1)-1)*n_y*n_x*n_y+(curr_pose(2)-1)*n_x*n_y+(curr_pose(3)-1)*n_y+curr_pose(4);

while (reach_state_target~=curr_pos_idx)
    U_x = find(Cont(curr_pos_idx,:));

    distances = zeros(1, size(U_x, 2));
    next_possible_poses = zeros(size(U_x, 2), 4);

    for idx=1:size(U_x, 2)
        inputs = get_composed_inputs(U_x(idx), n_r, n_inputs, inputs_data);
        
        n_x1 = curr_pose(1)+bound_u(inputs(2), 1);
        n_y1 = curr_pose(2)+bound_u(inputs(2), 2);
        n_x2 = curr_pose(3)+bound_u(inputs(1), 1);
        n_y2 = curr_pose(4)+bound_u(inputs(1), 2);
        
        next_possible_poses(idx, :) = [n_x1, n_y1, n_x2, n_y2];
        distances(idx) = norm([n_x1 n_y1] - [t_r1(1) t_r1(2)]) + norm([n_x2, n_y2] - [t_r2(1) t_r2(2)]);
    end

    [min_dist, index] = min(distances);
    curr_pose = next_possible_poses(index, :);
    curr_pos_idx = (curr_pose(1)-1)*n_y*n_x*n_y+(curr_pose(2)-1)*n_x*n_y+(curr_pose(3)-1)*n_y+curr_pose(4);
    poses = cat(1, poses, curr_pose);
    clear next_possible_poses  distances;
end

size(poses, 1)

for i=1:size(poses, 1)
    %%% marker plots
    plot(poses(i, 1), poses(i, 2),'x')
    hold on
    plot(poses(i, 3), poses(i, 4),'o')
    hold on

    %%% line plots
    plot(poses(1:i, 1), poses(1:i, 2))
    hold on
    plot(poses(1:i, 3), poses(1:i, 4))
    
    %%% graph properties
    xlim([1 n_lim])
    ylim([1 n_lim])
    grid on
    grid minor
    xlabel('x')
    ylabel('y')
    pause(0.1)
    
    if i ~= size(poses, 1)
        clf
    end
end

