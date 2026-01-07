clear all;
close all;

%%%% FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), x(k+1)=x(k)+u_y(k) %%%%

%% Individual Robot data
r = 1;

% states
states = 2;
s_eta = [15, 15];

% Constraints on state space
lb_states = [1, 1];
ub_states = [16, 16];

bound_x = [lb_states(1) ub_states(1)];
bound_y = [lb_states(2) ub_states(2)];
bound = [bound_x; bound_y];

% size of the interval
d_x = (bound_x(:,2)-bound_x(:,1))./(s_eta(1));
d_y = (bound_y(:,2)-bound_y(:,1))./(s_eta(2));
d_states = [d_x; d_y];

% inputs
inputs = 1;
i_eta = 8;

% All states for one robot
STATES = states + inputs;

% Constraints on the input space
lb_inputs = 1;
ub_inputs = 8;

bound_u = [-2 0; -1 0; 1 0; 2 0; 0 -2; 0 -1; 0 1; 0 2];

%% Total robot data
n_r = 2;

% states
n_states = [states; states];
n_s_eta = [s_eta; s_eta];

% inputs
n_inputs = [inputs; inputs];
n_i_eta = [i_eta; i_eta];

total_states = sum(n_states);
total_inputs = sum(n_inputs);
all_states = total_states + total_inputs;

targets = [[ 1,  1,  2,  2]; 
           [14, 14, 15, 15]; 
           [ 1, 14,  2, 15]];
f_targets = [];
for tar=1:size(targets, 1)
    f_targets = cat(1, f_targets, xtoX(targets(tar, :), d_states, bound));
end

obstacles = [[3.5, 3.5, 6.5, 6.5]; [8.5, 8.5, 11.5, 11.5]];
f_obstacles = [];
for obs=1:size(obstacles, 1)
    f_obstacles = cat(1, f_obstacles, xtoX(obstacles(obs, :), d_states, bound));
end

%% Saperate Controlled Delta for all robots
tic

disp(' ')
disp('[Barrier Composition]: Step 0- Get the Controlled Delta for all the robots.')
disp(' ')

controlled_delta = cell(1, n_r);
target_data = cell(1, n_r);
for R=1:n_r
    [delta, win_domain, T] = SimpleDiscreteSystem(r, states, inputs, lb_states, ub_states, lb_inputs, ub_inputs, s_eta, i_eta, targets(R, :), obstacles);
    controlled_delta{R} = find(delta);
    target_data{R} = find(T);
end

toc

%% Barrier Certificate = norm(x_i-x_j)-d-L*eta_max %%%%%
d=3; % Distance to be maintained
L=1; % Parameter for abstraction of barrier
eta_max = max((bound_x(2)-bound_x(1)+1)/s_eta(1), (bound_y(2)-bound_y(1)+1)/s_eta(2));

%% Barrier Composition of Controlled system
tic

disp(' ')
disp('[Barrier Composition]: Step 1- Computation of the abstraction.')
disp(' ')

p0 = waitbar(0, 'Abstraction in progress...');
Delta = logical(sparse((prod(s_eta)*prod(i_eta)).^n_r, prod(s_eta).^n_r));

D = controlled_delta;
[D{:}] = ndgrid(controlled_delta{:});
delta_idxs = cell2mat(cellfun(@(m)m(:), D, 'uni', 0));

points_data = cell(n_r, 2);
for d_idx=1:size(delta_idxs, 1)
    waitbar(d_idx/size(delta_idxs, 1), p0, sprintf('Computing the abstraction of the source subsystem - %2.2f', d_idx/size(delta_idxs, 1)))
    for R=1:n_r
        [r_c, r_s] = ind2sub([prod(s_eta)*prod(i_eta), prod(i_eta)], delta_idxs(d_idx, R));
        points = get_composed_current_pose(r_c, 1, states, inputs, s_eta, i_eta);
        s_points = get_composed_states(r_s, 1, states, s_eta);
        points_data{R, 1} = points(end:-1:1);
        points_data{R, 2} = s_points(end:-1:1);
    end
    
    possible_ids = nchoosek(1:n_r, 2);
    condition_data = zeros(size(possible_ids, 1), 2);
    for idx=1:size(possible_ids, 1)
        indicies = possible_ids(idx, :);
        r1 = points_data{indicies(1), :};
        r2 = points_data{indicies(2), :};
        condition_data(idx, :) = [norm(r1(1) - r2(1), inf), norm(r1(2) - r2(2), inf) - norm(r1(1) - r2(1), inf)];
    end

    if (all(condition_data(:, 1)-d-L*eta_max*0.5>=0))
        if (all(condition_data(:, 1) >= -0.9*(condition_data(:, 1)-d-L*eta_max*0.5)))
            points = points_data{:, 1};
            s_points = points_data{:, 2};
            i = get_composed_pose_index([points(end:-1:1, STATES), points(end:-1:1, STATES:-1:1)], n_r, n_states, n_inputs, n_s_eta, n_i_eta);
            j = get_composed_state_index([s_points(end:-1:1, end:-1:1)], n_r, n_states, n_s_eta);
            Delta(i, j) = 1;
        end
    end
end


% for d1_idx=1:size(delta1_idxs, 1)
%     waitbar(d1_idx/size(delta1_idxs, 1), p2, sprintf('Computing the abstraction of the source subsystem - %2.2f', d1_idx/size(delta1_idxs, 1)))
%     [r1_c, r1_s] = ind2sub(size(delta1), delta1_idxs(d1_idx));
%     points1 = get_composed_current_pose(r1_c, r, states, inputs, s_eta, i_eta);
%     s_points1 = get_composed_states(r1_s, r, states, s_eta);
%     for d2_idx=1:size(delta2_idxs, 1)
%         [r2_c, r2_s] = ind2sub(size(delta2), delta2_idxs(d2_idx));
%         points2 = get_composed_current_pose(r2_c, r, states, inputs, s_eta, i_eta);
%         s_points2 = get_composed_states(r2_s, r, states, s_eta);
%        
%         if(norm([points1(3:-1:2)] - [points2(3:-1:2)])-d-L*eta_max*0.5>=0)
%             B_prime = norm([s_points1(2:-1:1)] - [s_points2(2:-1:1)]) - norm([points1(3:-1:2)] - [points2(3:-1:2)]);
%             if (B_prime >= -0.9*(norm([points1(3:-1:2)] - [points2(3:-1:2)])-d-L*eta_max*0.5))
%                 i = get_composed_pose_index([points2(1), points1(1), points2(2:3), points1(2:3)], n_r, n_states, n_inputs, n_s_eta, n_i_eta);
%                 j = get_composed_state_index([s_points2, s_points1], n_r, n_states, n_s_eta);
%                 Delta(i, j) = 1;
%             end
%         end
%     end
% end
close(p0)
toc


%% Controller Synthesis
tic

disp(' ')
disp('[Barrier Composition]: Step 2 - Controller Synthesis.')
disp(' ')

% create the matrix structure (defined only with 0,1)
Controller_syn = false(prod(s_eta).^n_r, prod(i_eta).^n_r);
succ_idxs = find(Delta);

for succ_idx=1:size(succ_idxs, 1)
%     waitbar(succ_idx/size(succ_idxs, 1), p2, sprintf('Computing the Controller Synthesis - %2.2f', succ_idx/size(succ_idxs, 1)))
    [r_c, r_s] = ind2sub(size(Delta), succ_idxs(succ_idx));
    points = get_composed_current_pose(r_c, n_r, n_states, n_inputs, n_s_eta, n_i_eta);
    i = get_composed_state_index([points(total_inputs+1:all_states)], n_r, n_states, n_s_eta);
    h = get_composed_input_index([points(1:total_inputs)], n_r, n_inputs, n_i_eta);
    Controller_syn(i, h) = 1;
end

%% Reachability
disp(' ')
disp('Step 3: Reachability')
disp(' ')

Z = logical(sparse(prod(s_eta).^n_r, prod(i_eta).^n_r));
Z(1:prod(s_eta)^n_r, 1:prod(i_eta)^n_r) = 1;

Zn = logical(sparse(prod(s_eta).^n_r, prod(i_eta).^n_r));

%%% Reachability Specification.
T = logical(sparse(prod(s_eta).^n_r, prod(i_eta).^n_r));
Tn = target_data;
[Tn{:}] = ndgrid(target_data{:});
Tn_idxs = cell2mat(cellfun(@(m)m(:), Tn, 'uni', 0));

for t_idx=1:size(Tn_idxs, 1) % compute the whole interval contained into the extremal intervals
    for R=1:n_r
        [st, ~] = ind2sub(size(target_data{1}), Tn_idxs(t_idx, R));
    end
    
    
    for t2_idx=1:size(T2_idxs, 2)
        [st2, ~] = ind2sub(size(T2), T2_idxs(t2_idx));
        i = (st1 - 1)*prod(s_eta) + st2;
        T(i, :) = 1;
    end
end
reach_state_idxs = find(T);
[reach_state, ~] = ind2sub(size(T), reach_state_idxs);
reach_state = unique(reach_state);

Zn_check = {};
J_check = logical(sparse(prod(s_eta).^n_r, 0));

iter=0;
while (nnz(Z) ~= nnz(Zn))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter)                   
    
    Z = Zn;
    pres = get_pre2(Z, [], Delta);
    for i=1:length(pres)
        inputs = get_composed_inputs(pres(i), n_r, n_inputs, n_i_eta, 1);
        Zn(inputs(1), inputs(2)) = 1;
    end
    Zn = Zn | T; 

    Z_diff = xor(Zn, Z);     
    Z_diff_idxs = find(Z_diff);
    for i=1:size(Z_diff_idxs, 1)
        [r, ~] = ind2sub(size(Z_diff), Z_diff_idxs(i));
        if(all(J_check(r, :) == 0))
            J_check(r, iter) = 1;
        end
    end
    Zn_check{iter} = Z_diff;
end
Controller = Zn;
toc

%% Converting the Controller into Transition System and Computing the Winning Deomain
disp(' ')
disp('[Barrier Composition]: Step 4 - Comuputation of Transition System and Winnig Domain')
disp(' ')

controlled_delta_idxs = find(Controller);
Winning_Domain = zeros(size(controlled_delta_idxs, 1), 3);

for idx=1:size(controlled_delta_idxs, 1)
    [pre, input] = ind2sub(size(Controller), controlled_delta_idxs(idx));
    points = get_composed_states(pre, n_r, n_states, n_s_eta);
    Winning_Domain(idx, :) = [points(2), points(1), input];
end

%% Simulation
% Current pose index
X_curr = [19.5, 19.5, 1.5, 1.5];
curr_pose1 = XtoI(X_curr(1:2), d_states, bound, 'floor');
curr_pose2 = XtoI(X_curr(3:end), d_states, bound, 'floor');

curr_pose = [curr_pose2(end:-1:1), curr_pose1(end:-1:1)];
curr_pos_idx = get_composed_state_index(curr_pose, n_r, n_states, n_s_eta);

% for plot
poses = X_curr;

while (~ismember(curr_pos_idx, reach_state))
    j_idx = find(J_check(curr_pos_idx, :));
    controller = Zn_check{j_idx};
    controller_idxs = find(controller(curr_pos_idx, :));
    
    distances = zeros(1, length(controller_idxs));
    next_possible_poses = zeros(length(controller_idxs), total_states);

    for idx=1:length(controller_idxs)
        inputs_idx = controller_idxs(idx);
        inputs = get_composed_inputs(inputs_idx, n_r, n_inputs, n_i_eta, 0);

        X_new(1:2) = X_curr(1:2) + bound_u(inputs(2), :);
        X_new(3:4) = X_curr(3:4) + bound_u(inputs(1), :);
        
        next_possible_poses(idx, :) = X_new;
        distances(idx) = norm([X_new(1:2)] - [target1(1) target1(2)]) + norm([X_new(3:4)] - [target2(1) target2(2)]);
    end

    [~, index] = min(distances);
    X_curr = next_possible_poses(index, :);
    curr_pose1 = XtoI(X_curr(1:2), d_states, bound, 'floor');
    curr_pose2 = XtoI(X_curr(3:end), d_states, bound, 'floor');

    curr_pose = [curr_pose2(end:-1:1), curr_pose1(end:-1:1)];
    curr_pos_idx = get_composed_state_index(curr_pose, n_r, n_states, n_s_eta);
    poses = cat(1, poses, X_curr);
    clear next_possible_poses  distances;
end

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

    %%% Plot the target and obstacles
    plot_objects(f_targets, f_obstacles)
    
    %%% graph properties
    xlim([lb_states(1)-2, ub_states(1)+2])
    ylim([lb_states(2)-2, ub_states(2)+2])
    grid on
    grid minor
    xlabel('x')
    ylabel('y')
    pause(0.3)
    
    if i ~= size(poses, 1)
        clf
    end
end

function plot_objects(n_target, n_obstacles)
    colors=get(groot,'DefaultAxesColorOrder');
    alpha=0.2;

    n_obs = size(n_obstacles, 1);
    for i=1:n_obs
        v = [[n_obstacles(i, 1), n_obstacles(i, 2)]; [n_obstacles(i, 3), n_obstacles(i, 2)]; [n_obstacles(i, 1), n_obstacles(i, 4)]; [n_obstacles(i, 3), n_obstacles(i, 4)]];
        patch('vertices', v, 'faces', [1 2 4 3], 'facea', alpha, 'facec', colors(1,:), 'edgec', colors(1,:))
    end
    
    n_tar = size(n_target, 1);
    for i=1:n_tar
        v = [[n_target(i, 1), n_target(i, 2)]; [n_target(i, 3), n_target(i, 2)]; [n_target(i, 1), n_target(i, 4)]; [n_target(i, 3), n_target(i, 4)]];
        patch('vertices', v, 'faces', [1 2 4 3], 'facea', alpha, 'facec', colors(2,:), 'edgec', colors(2,:))
    end
end