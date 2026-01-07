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

% Constraints on the input space
lb_inputs = 1;
ub_inputs = 8;

bound_u = [-2 0; -1 0; 1 0; 2 0; 0 -2; 0 -1; 0 1; 0 2];


target1 = [1, 1, 2, 2];
target2 = [14, 14, 15, 15];

targets = [target1; target2];
f_targets = [];
for tar=1:size(targets, 1)
    f_targets = cat(1, f_targets, xtoX(targets(tar, :), d_states, bound));
end

obstacles = [[3.5, 3.5, 6.5, 6.5]; [8.5, 8.5, 11.5, 11.5]];
f_obstacles = [];
for obs=1:size(obstacles, 1)
    f_obstacles = cat(1, f_obstacles, xtoX(obstacles(obs, :), d_states, bound));
end

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

% Distance to be maintained
L = 1;
d = 3;

%% Abstraction
tic

disp(' ')
disp('[Monolithic]: Step 1- Computation of the abstraction.')
disp(' ')

p1 = waitbar(0,'Abstraction in progress...');
Delta = logical(sparse((prod(s_eta)*prod(i_eta)).^n_r, prod(s_eta).^n_r));

% Compute the automata
for i1=1:n_s_eta(1, 1)
    waitbar(i1/n_s_eta(1, 1), p1, sprintf('Computing the abstraction of the source subsystem - %2.2f', i1/n_s_eta(1, 1)))
    for i2=1:n_s_eta(1, 2)
        for i3=1:n_s_eta(2, 1)
            for i4=1:n_s_eta(2, 2)
                for h1=1:n_i_eta(1, 1)
                    for h2=1:n_i_eta(2, 1)
                        X1_state = ItoX([i1, i2], d_states, bound);
                        succ1 = X1_state + bound_u(h1, :)';
                        X2_state = ItoX([i3, i4], d_states, bound);
                        succ2 = X2_state + bound_u(h2, :)';
                        if((succ1(1) >= bound(1,1))&&(succ1(1) <= bound(1,2)) && (succ1(2) >= bound(2,1))&&(succ1(2) <= bound(2,2)) && (succ2(1) >= bound(1,1))&&(succ2(1) <= bound(1,2)) && (succ2(2) >= bound(2,1))&&(succ2(2) <= bound(2,2)))
                            X1_succ = XtoI(succ1, d_states, bound, 'floor');
                            X2_succ = XtoI(succ2, d_states, bound, 'floor');
                            i = get_composed_pose_index([h2, h1, i4, i3, i2, i1], n_r, n_states, n_inputs, n_s_eta, n_i_eta);
                            j = get_composed_state_index([X2_succ(end:-1:1), X1_succ(end:-1:1)], n_r, n_states, n_s_eta);
                            Delta(i, j) = 1;
                        end
                    end
                end
            end    
        end
    end
end

close(p1);
toc

%% DEFINITION OF OBSTACLE SET %%%%
tic 

disp(' ')
disp('[Monolithic]: Obstacle set.')
disp(' ')

obstacle_set = logical(sparse(prod(s_eta), prod(i_eta)));
n_obs = size(obstacles, 1);
for o=1:n_obs
    min_ids = XtoI(obstacles(o, 1:states), d_states, bound, 'floor');
    max_ids = XtoI(obstacles(o, states+1:end), d_states, bound, 'ceil');
    for ip1=min_ids(1):max_ids(1)
        for ip2=min_ids(2):max_ids(2)
            obstacle_set((ip1 - 1)*s_eta(2) + ip2, :) = 1;
        end
    end
end

free_set = ~obstacle_set;
free_idxs = find(free_set);
[obs_st, ~] = ind2sub(size(obstacle_set), free_idxs);
obs_st = unique(obs_st);

n_free_set = logical(sparse(prod(s_eta).^n_r, prod(i_eta).^n_r));
for r1=1:size(obs_st, 1)
    for r2=1:size(obs_st, 1)
        n_free_set((obs_st(r1)-1)*prod(s_eta) + obs_st(r2), :) = 1;
    end
end

n_obstacle_set = ~n_free_set;
for i=1:n_s_eta(1, 1)
    for j=1:n_s_eta(1, 2)
        for k=1:n_s_eta(2, 1)
            for l=1:n_s_eta(2, 2)
                if(norm([i, j] - [k, l])-L*0.5-d<0)
                    idx = get_composed_state_index([l, k, j, i], n_r, n_states, n_s_eta);
                    n_obstacle_set(idx, :) = 1;
                end
            end
        end
    end
end
obs_state_idxs = find(n_obstacle_set);
[obs_state, ~] = ind2sub(size(n_obstacle_set), obs_state_idxs);
obs_state = unique(obs_state);

toc

%% Controller Synthesis
tic

disp(' ')
disp('[Monolithic]: Step 2 - Controller Synthesis.')
disp(' ')

% create the matrix structure (defined only with 0,1) 
Controller_syn = false(prod(s_eta).^n_r, prod(i_eta).^n_r);
succ_idxs = find(Delta);

for succ_idx=1:size(succ_idxs, 1)
    [r_c, r_s] = ind2sub(size(Delta), succ_idxs(succ_idx));
    points = get_composed_current_pose(r_c, n_r, n_states, n_inputs, n_s_eta, n_i_eta);
    i = get_composed_state_index([points(total_inputs+1:all_states)], n_r, n_states, n_s_eta);
    h = get_composed_input_index([points(1:total_inputs)], n_r, n_inputs, n_i_eta);
    Controller_syn(i, h) = 1;
end

%% Reachability
disp(' ')
disp('[Monolithic]: Step 3 - Reachability')
disp(' ')

Z = logical(sparse(prod(s_eta).^n_r, prod(i_eta).^n_r));
Z(1:prod(s_eta)^n_r, 1:prod(i_eta)^n_r) = 1;

Zn = logical(sparse(prod(s_eta).^n_r, prod(i_eta).^n_r));

%%% Reachability Specification.
T = logical(sparse(prod(s_eta).^n_r, prod(i_eta).^n_r));

min1_ids = XtoI(targets(1, 1:states), d_states, bound, 'floor');
max1_ids = XtoI(targets(1, states+1:end), d_states, bound, 'ceil');
min2_ids = XtoI(targets(2, 1:states), d_states, bound, 'floor');
max2_ids = XtoI(targets(2, states+1:end), d_states, bound, 'ceil');
for ip1=min1_ids(1):max1_ids(1)
    for ip2=min1_ids(2):max1_ids(2)
        for ip3=min2_ids(1):max2_ids(1)
            for ip4=min2_ids(2):max2_ids(2)
                i = get_composed_state_index([ip4, ip3, ip2, ip1], n_r, n_states, n_s_eta);            
                T(i, :) = 1;                                
            end
        end
    end
end

reach_state_idxs = find(T);
[reach_state, ~] = ind2sub(size(T), reach_state_idxs);
reach_state = unique(reach_state);

T(reach_state, lb_inputs(1)^n_r:ub_inputs(1)^n_r) = 1;

Zn_check = {};
J_check = logical(sparse(prod(s_eta).^n_r, 0));

iter=0;
while (nnz(Z) ~= nnz(Zn))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter)
    
    Z = Zn;
    pres = get_pre2(Z, obs_state, Delta);
    for i=1:length(pres)
        inputs = get_composed_inputs(pres(i), n_r, n_inputs, n_i_eta, 1);
        Zn(inputs(1), inputs(2)) = 1;
    end
    Zn = Zn & (~n_obstacle_set);
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
disp('[Monolithic]: Step 4 - Comuputation of Transition System and Winnig Domain')
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
X_curr = [14.5, 14.5, 1.5, 1.5];
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