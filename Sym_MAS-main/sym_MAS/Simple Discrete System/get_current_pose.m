%%%% index to current pose
function [x, y, u] = get_current_pose(index, n_x, n_y, n_u)
    if(rem(index, n_u)==0)
        u = n_u;
    else
        u = rem(index, n_u);
    end
    
    index = (index-u)/n_u+1;
    if(rem(index, n_y)==0)
        y = n_y;
    else
        y = rem(index, n_y);
    end
    x = (index-y)/n_y+1;
end