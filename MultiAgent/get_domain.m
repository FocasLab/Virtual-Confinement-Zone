function domain = get_domain(Controller)
    c_idxs = find(Controller);
    [pre, ~] = ind2sub(size(Controller), c_idxs);
    uni_domain = unique(pre);
    domain = size(uni_domain, 1);
end