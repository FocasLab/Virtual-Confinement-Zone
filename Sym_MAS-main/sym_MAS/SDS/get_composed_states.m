function[states] = get_composed_states(index, n_r, n_states, states_data)
    total_states = sum(n_states);
    
    % Data for iterations
    subs = zeros(1, total_states+1);
    dividors = ones(1, 1);
    adds = cat(2, 0, ones(1, total_states - 1));
    states = zeros(1, total_states);

    for r=n_r:-1:1
        for i=n_states(r):-1:1
            dividors = cat(2, dividors, states_data(r, i));
        end
    end

    for idx=1:total_states
        index = (index - subs(idx))/dividors(idx) + adds(idx);
        if(rem(index, dividors(idx+1))==0)
            states(idx) = dividors(idx+1);
        else
            states(idx) = rem(index, dividors(idx+1));
        end
        subs(idx+1) = states(idx);
    end
end