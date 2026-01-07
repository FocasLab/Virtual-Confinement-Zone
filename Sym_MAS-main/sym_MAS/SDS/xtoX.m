function range = xtoX(data, d_states, bound)
    states = length(d_states);
    min_ids = XtoI(data(:, 1:states), d_states, bound, 'floor');
    max_ids = XtoI(data(:, states+1:end), d_states, bound, 'ceil');
    i_data = [min_ids', max_ids'];
    lb_data = ItoX(i_data(:, 1:states), d_states, bound) - d_states./2;
    ub_data = ItoX(i_data(:, states+1:end), d_states, bound) + d_states./2;
    range = [lb_data', ub_data'];
end