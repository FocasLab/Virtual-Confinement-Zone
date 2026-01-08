function [preZn] = get_pre2(Z, Obs_states, Delta)
    % Get the indicies of the succ
    Z_idxs = find(Z);
    [succ, ~] = ind2sub(size(Z), Z_idxs);
    uni_succ = setdiff(unique(succ), Obs_states);
    
    % Get the indicies of the pre
    preZn_idxs = find(Delta(:, uni_succ));
    [preZn, ~] = ind2sub(size(Delta), preZn_idxs);
    preZn = unique(preZn);
end