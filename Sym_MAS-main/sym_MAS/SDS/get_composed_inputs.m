function[inputs] = get_composed_inputs(index, n_r, n_inputs, inputs_data, flag)
    total_inputs = sum(n_inputs);
    
    % Data for iterations
    subs = zeros(1, total_inputs+1);
    dividors = ones(1, 1);
    adds = cat(2, 0, ones(1, total_inputs));
    inputs = zeros(1, total_inputs);

    for r=n_r:-1:1
        for i=n_inputs(r):-1:1
            dividors = cat(2, dividors, inputs_data(r, i));
        end
    end

    for idx=1:total_inputs
        if(rem(index, dividors(idx+1))==0)
            inputs(idx) = dividors(idx+1);
        else
            inputs(idx) = rem(index, dividors(idx+1));
        end
        subs(idx+1) = inputs(idx);
        index = (index - subs(idx+1))/dividors(idx+1) + adds(idx+1);
    end
    
    if(flag==1)
        inputs = [index, get_composed_input_index(inputs, n_r, n_inputs, inputs_data)];
    end
end