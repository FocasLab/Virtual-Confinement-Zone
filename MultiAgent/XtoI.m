function [indexs] = XtoI(X, d_states, bound, type)
    X = reshape(X, size(d_states));
    if(strcmp(type, 'ceil'))
        indexs = ceil((X - bound(:, 1))./d_states);
    else
        indexs = floor((X - bound(:, 1))./d_states) + ones(length(d_states), 1);
    end
end