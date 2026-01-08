%%%% index to compose current pose
function [points] = get_composed_current_pose(index, n_r, n_states, n_inputs, states_data, inputs_data)
    % Total number of states
    total_states = sum(n_states) + sum(n_inputs);
     
    % Data for iterations
    subs = zeros(1, total_states+1);
    dividors = ones(1, 1);
    adds = cat(2, 0, ones(1, total_states - 1));
    points = zeros(1, total_states);
    
    for flag=1:2
        for r=n_r:-1:1
            if(flag==1)
                for i=n_inputs(r):-1:1
                    dividors = cat(2, dividors, inputs_data(r, i));
                end
            else
                for s=n_states(r):-1:1
                    dividors = cat(2, dividors, states_data(r, s));
                end
            end
        end
    end
    
    for idx=1:total_states
        index = (index - subs(idx))/dividors(idx) + adds(idx);
        if(rem(index, dividors(idx+1))==0)
            points(idx) = dividors(idx+1);
        else
            points(idx) = rem(index, dividors(idx+1));
        end
        subs(idx+1) = points(idx);
    end
    
end