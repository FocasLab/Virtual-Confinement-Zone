%%%% compose states to index
function [index] = get_composed_state_index(sub_data, n_r, n_states, n_s_eta)
    % Total number of states
    total_states = sum(n_states);
     
    % Data for iterations
    multipliers = ones(1, 1);
    subs = cat(2, 0, ones(1, total_states - 1));
    index = 0;
    
    for r=n_r:-1:1
        for s=n_states(r):-1:1
            multipliers = cat(2, multipliers, n_s_eta(r, s));
        end
    end

    for idx=1:total_states
        index = index + (sub_data(idx) - subs(idx))*prod(multipliers(1:idx));
    end
end