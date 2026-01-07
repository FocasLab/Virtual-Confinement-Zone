%%%% index to successor pose
function [x, y] = get_successor_pose(index, n_x, n_y)
    if(rem(index, n_y)==0)
        y = n_y;
    else
        y = rem(index, n_y);
    end
    x = (index-y)/n_y+1;
end