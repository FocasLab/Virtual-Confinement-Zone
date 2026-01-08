function [X] = ItoX(indexs, d_states, bound)
    indexs = reshape(indexs, size(d_states));
    X = bound(:, 1) + (indexs-1).*d_states + d_states./2.0;
end