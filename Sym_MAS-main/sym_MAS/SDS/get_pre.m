function [preZn] = get_pre(Z, Controller, n_i_eta, Delta)
    controller_idxs = find(Controller);
    preZn = [];
    for idx=1:size(controller_idxs, 1)
        [pre, input] = ind2sub(size(Controller), controller_idxs(idx));
        successor = find(Delta((pre - 1)*prod(n_i_eta) + input, :));
        if(ismember(successor, Z(:, 2)))
            preZn = cat(1, preZn, [input, pre]);
        end
    end
end