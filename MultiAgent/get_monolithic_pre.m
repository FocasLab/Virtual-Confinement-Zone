function [preZn] = get_monolithic_pre(Z, Obstacle_set, Delta)
    Z_idxs = find(Z);
    [succ, ~] = ind2sub(size(Z), Z_idxs);
    preZn_idxs = find(Delta(:, unique(succ)));
    [preZn, ~] = ind2sub(size(Delta), preZn_idxs);
    preZn = preZn & (~Obstacle_set);
end