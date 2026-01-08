%%%% compose inputs to index
function [index] = get_composed_input_index(sub_data, n_r, n_inputs, n_i_eta)
    % Total number of states
    total_inputs = sum(n_inputs);
     
    % Data for iterations
    multipliers = ones(1, 1);
    subs = cat(2, 0, ones(1, total_inputs - 1));
    index = 0;
    
    for r=n_r:-1:1
        for s=n_inputs(r):-1:1
            multipliers = cat(2, multipliers, n_i_eta(r, s));
        end
    end

    for idx=1:total_inputs
        index = index + (sub_data(idx) - subs(idx))*prod(multipliers(1:idx));
    end
end