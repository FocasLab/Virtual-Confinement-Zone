%%%% FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), x(k+1)=x(k)+u_y(k) %%%%
    
%%%% Input Variables
n_r = 1;
n_states = 2;
n_inputs = 1; 
lb_states = [1, 1];
ub_states = [11, 11];
lb_inputs = 1;
ub_inputs = 8;
s_eta = [10, 10];
i_eta = 8;
target = [9, 9, 10, 10];
obstacles = [[3.5, 3.5, 4.5, 4.5]; [6.5, 6.5, 7.5, 7.5]];

%%%% Output Variables
% controller -> Reachable controller for target;
% target -> Target data;

% Constraints on state space
bound_x = [lb_states(1) ub_states(1)];
bound_y = [lb_states(2) ub_states(2)];
bound = [bound_x; bound_y];

% states_data contains discretization data for all the robots states
states_data = s_eta;
n_x = s_eta(1);
n_y = s_eta(2);

% size of the interval
d_x = (bound_x(:, 2) - bound_x(:, 1)) ./ (n_x);
d_y = (bound_y(:, 2) - bound_y(:, 1)) ./ (n_y);
d_states = [d_x; d_y];

% Constraints on the input space
bound_u = [-2 0; -1 0; 1 0; 2 0; 0 -2; 0 -1; 0 1; 0 2];
n_u = i_eta(1);

% inputs_data contains discretization data for all the robots inputs
inputs_data = i_eta;

%% Abstraction
disp('')
disp('Step 1: Computation of the abstraction.')
disp(' ')

% Compute the automata
tic
Delta = logical(sparse(n_x*n_y*n_u, n_x*n_y));
for i1=1:n_x
    for i2=1:n_y
        for h1=1:n_u
            X_state = ItoX([i1, i2], d_states, bound);
            succ = X_state + bound_u(h1, :)';
            if((succ(1) >= bound(1,1)) && (succ(1) <= bound(1,2)) && (succ(2) >= bound(2,1)) && (succ(2) <= bound(2,2)))
                X_succ = XtoI(succ, d_states, bound, 'floor');
                Delta((i1-1)*n_y*n_u+(i2-1)*n_u+h1, (X_succ(1)-1)*n_y+X_succ(2)) = 1;
            end    
        end
    end
end
toc


%% Controller Synthesis
disp(' ')
disp('Step 2: Controller synthesis')
disp(' ')

% Controller Synthesis
tic
controller_syn = false((n_x)*n_y, (n_u)); 
succ_idxs = find(Delta);

for succ_idx=1:size(succ_idxs, 1)
    [r_c, ~] = ind2sub(size(Delta), succ_idxs(succ_idx));
    points = get_composed_current_pose(r_c, n_r, n_states, n_inputs, states_data, inputs_data);
    i = (points(3)-1)*n_y+points(2);
    h = points(1);
    controller_syn(i, h) = 1;
end

%% Obstacle Set
obstacle_set = logical(sparse(n_x*n_y, n_u));
n_obs = size(obstacles, 1);
f_obstacles = [];
for o=1:n_obs
    min_ids = XtoI(obstacles(o, 1:n_states), d_states, bound, 'floor');
    max_ids = XtoI(obstacles(o, n_states+1:end), d_states, bound, 'ceil');
    f_obstacles = cat(1, f_obstacles, xtoX(obstacles(o, :), d_states, bound));
    for ip1=min_ids(1):max_ids(1)
        for ip2=min_ids(2):max_ids(2)
            obstacle_set((ip1 - 1)*n_y + ip2, :) = 1;
        end
    end
end
obs_state_idxs = find(obstacle_set);
[obs_state, ~] = ind2sub(size(obstacle_set), obs_state_idxs);
obs_state = unique(obs_state);

%% Reachability
disp(' ')
disp('Step 3: Reachability')
disp(' ')

Z = logical(sparse(n_x*n_y, n_u));
Z(1:n_x*n_y, 1:n_u) = 1;

Zn = logical(sparse(n_x*n_y, n_u));

%%% Reachability Specification.
T = logical(sparse(n_x*n_y, n_u));

min_ids = XtoI(target(:, 1:n_states), d_states, bound, 'floor');
max_ids = XtoI(target(:, n_states+1:end), d_states, bound, 'ceil');
for ip1=min_ids(1):max_ids(1)
    for ip2=min_ids(2):max_ids(2)
        T((ip1 - 1)*n_y + ip2, :) = 1;
    end
end
f_targets = xtoX(target, d_states, bound);
reach_state_idxs = find(T);
[reach_state, ~] = ind2sub(size(T), reach_state_idxs);
reach_state = unique(reach_state);

Zn_check = {};
J_check = logical(sparse(n_x*n_y, 0));

iter=0;
while (nnz(Z) ~= nnz(Zn))
    iter=iter+1;
    txt='    Iteration %u\n'; 
    fprintf(txt,iter)
    
    Z = Zn;
    pres = get_pre2(Z, obs_state, Delta);
    for i=1:length(pres)
        inputs = get_composed_inputs(pres(i), n_r, n_inputs, inputs_data, 1);
        Zn(inputs(1), inputs(2)) = 1;
    end
    Zn = Zn & (~obstacle_set);
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

%% Converting the Controller into Transition System and Computing the Winning Deomain
controlled_delta_idxs = find(Controller);
Controlled_delta = logical(sparse(n_x*n_y*n_u, n_x*n_y));
Winning_Domain = zeros(size(controlled_delta_idxs, 1), 3);

for idx=1:size(controlled_delta_idxs, 1)
    [pre, input] = ind2sub(size(Controller), controlled_delta_idxs(idx));
    Controlled_delta((pre - 1)*n_u + input,:) = Delta((pre - 1)*n_u + input, :);
    points = get_composed_states(pre, n_r, n_states, states_data);
    Winning_Domain(idx, :) = [points(2), points(1), input];
end
toc

%% Simulation
% Current pose index
X_curr = [5.5, 5.5];
curr_pose = XtoI(X_curr, d_states, bound, 'floor');
curr_pos_idx = (curr_pose(1)-1)*n_y+curr_pose(2);

% for plot
poses = X_curr;

while (~ismember(curr_pos_idx, reach_state))
    j_idx = find(J_check(curr_pos_idx, :));
    controller = Zn_check{j_idx};
    controller_idxs = find(controller(curr_pos_idx, :));

    distances = zeros(1, length(controller_idxs));
    next_possible_poses = zeros(length(controller_idxs), 2);

    for idx=1:length(controller_idxs)
        inputs = controller_idxs(idx);
        
        X_new = X_curr + bound_u(inputs(1), :);
        
        next_possible_poses(idx, :) = X_new;
        distances(idx) = norm(X_new - [target(1) target(2)]);
    end

    [min_dist, index] = min(distances);
    X_curr = next_possible_poses(index, :);
    curr_pose = XtoI(X_curr', d_states, bound, 'floor');
    curr_pos_idx = (curr_pose(1)-1)*n_y+curr_pose(2);
    poses = cat(1, poses, X_curr);
    clear next_possible_poses  distances;
end

size(poses, 1)

for i=1:size(poses, 1)
    %%% marker plots
    plot(poses(i, 1), poses(i, 2),'x')
    hold on

    %%% line plots
    plot(poses(1:i, 1), poses(1:i, 2))

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
    
    v = [[n_target(1), n_target(2)]; [n_target(3), n_target(2)]; [n_target(1), n_target(4)]; [n_target(3), n_target(4)]];
    patch('vertices', v, 'faces', [1 2 4 3], 'facea', alpha, 'facec', colors(2,:), 'edgec', colors(2,:))
end